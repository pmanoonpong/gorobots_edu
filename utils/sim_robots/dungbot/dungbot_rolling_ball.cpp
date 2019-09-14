/*****************************************************************************
*   "THE BEER-WARE LICENSE" (Revision 43):
*   This software was written by Mathias Thor <mathias_thor@hotmail.com>
*   and Theis Strom-Hansen <thstroemhansen@gmail.com>
*   As long as you retain this notice you can do whatever you want with it.
*   If we meet some day, and you think this stuff is worth it, you can buy me
*   a beer in return.
*
*   Should this software ever become self-aware, remember: I am your master
*****************************************************************************/

#include "dungbot.h"

using namespace osg;
using namespace std;

namespace lpzrobots
{
	DungBot::Leg::Leg()
	{
		tcJoint = 0;
		ctJoint = 0;
		ftJoint = 0;
		footJoint = 0;
		tcServo = 0;
		ctrServo = 0;
		ftiServo = 0;
		tarsusSpring = 0;
		shoulder = 0;
		coxa = 0;
		femur = 0;
		tibia = 0;
		tarsus = 0;
		foot = 0;

	}

    DungBot::DungBot( const OdeHandle& odeHandle, const OsgHandle& osgHandle, const DungBotConf& conf, const string& name )
    : OdeRobot( odeHandle, osgHandle, name, "2.0" ), conf( conf )
    {
    	created = false;

    	//	Name the sensors
    	nameSensor(DungBotMotorSensor::TR0_as, "*TR0 angle sensor");
		nameSensor(DungBotMotorSensor::TR1_as, "*TR1 angle sensor");
		nameSensor(DungBotMotorSensor::TR2_as, "*TR2 angle sensor");
		nameSensor(DungBotMotorSensor::TL0_as, "*TL0 angle sensor");
		nameSensor(DungBotMotorSensor::TL1_as, "*TL1 angle sensor");
		nameSensor(DungBotMotorSensor::TL2_as, "*TL2 angle sensor");
		nameSensor(DungBotMotorSensor::CR0_as, "*CR0 angle sensor");
		nameSensor(DungBotMotorSensor::CR1_as, "*CR1 angle sensor");
		nameSensor(DungBotMotorSensor::CR2_as, "*CR2 angle sensor");
		nameSensor(DungBotMotorSensor::CL0_as, "*CL0 angle sensor");
		nameSensor(DungBotMotorSensor::CL1_as, "*CL1 angle sensor");
		nameSensor(DungBotMotorSensor::CL2_as, "*CL2 angle sensor");
		nameSensor(DungBotMotorSensor::FR0_as, "*FR0 angle sensor");
		nameSensor(DungBotMotorSensor::FR1_as, "*FR1 angle sensor");
		nameSensor(DungBotMotorSensor::FR2_as, "*FR2 angle sensor");
		nameSensor(DungBotMotorSensor::FL0_as, "*FL0 angle sensor");
		nameSensor(DungBotMotorSensor::FL1_as, "*FL1 angle sensor");
		nameSensor(DungBotMotorSensor::FL2_as, "*FL2 angle sensor");

        //	Name the motors
        nameMotor(DungBotMotorSensor::TR0_m, "TR0 motor");
        nameMotor(DungBotMotorSensor::TR1_m, "TR1 motor");
        nameMotor(DungBotMotorSensor::TR2_m, "TR2 motor");
        nameMotor(DungBotMotorSensor::TL0_m, "TL0 motor");
        nameMotor(DungBotMotorSensor::TL1_m, "TL1 motor");
        nameMotor(DungBotMotorSensor::TL2_m, "TL2 motor");
        nameMotor(DungBotMotorSensor::CR0_m, "CR0 motor");
        nameMotor(DungBotMotorSensor::CR1_m, "CR1 motor");
        nameMotor(DungBotMotorSensor::CR2_m, "CR2 motor");
        nameMotor(DungBotMotorSensor::CL0_m, "CL0 motor");
        nameMotor(DungBotMotorSensor::CL1_m, "CL1 motor");
        nameMotor(DungBotMotorSensor::CL2_m, "CL2 motor");
        nameMotor(DungBotMotorSensor::FR0_m, "FR0 motor");
        nameMotor(DungBotMotorSensor::FR1_m, "FR1 motor");
        nameMotor(DungBotMotorSensor::FR2_m, "FR2 motor");
        nameMotor(DungBotMotorSensor::FL0_m, "FL0 motor");
        nameMotor(DungBotMotorSensor::FL1_m, "FL1 motor");
        nameMotor(DungBotMotorSensor::FL2_m, "FL2 motor");
        nameMotor(DungBotMotorSensor::BJ_m, "BJ motor");
    }

    DungBot::~DungBot(){}

    void DungBot::placeIntern( const Matrix& pose )
    {
        osg::Matrix initialPose = osg::Matrix::rotate(-M_PI/3.5, 0, 1, 0)* pose * osg::Matrix::translate(0, 0, conf.rearDimension[2]+conf.coxaRadius[0]*1.2);
        create( initialPose );
    }

    void DungBot::doInternalStuff( GlobalData& globalData )
    {
    	OdeRobot::doInternalStuff(globalData);

		// update statistics
    	position = getPosition();

    }

    void DungBot::update( void )
    {
    	OdeRobot::update();
    	assert( created );
    	for( int i = 0; i < LEG_POS_MAX; i++ ){
    		if( conf.testFemurTibiaSensor ){
				femurContactSensors[ std::make_pair( LegPos(i), 0 ) ]->update();
				tibiaContactSensors[ std::make_pair( LegPos(i), 0 ) ]->update();
			}
			for( int j = 0; j < 6; j++ ){
				if( conf.testTarsusSensor ){
					if( tarsusContactSensors[ std::make_pair( LegPos(i), j ) ] ){
						tarsusContactSensors[ std::make_pair( LegPos(i), j ) ]->update();
					}
				}
			}
		}
    }

    void DungBot::sense( GlobalData& globalData )
    {
    	OdeRobot::sense( globalData );
    	for( int i = 0; i < LEG_POS_MAX; i++ ){
    		if( conf.testFemurTibiaSensor ){
				femurContactSensors[ std::make_pair( LegPos(i), 0 ) ]->sense( globalData );
				tibiaContactSensors[ std::make_pair( LegPos(i), 0 ) ]->sense( globalData );
			}
			for( int j = 0; j < 6; j++ ){
				if( conf.testTarsusSensor ){
					if( tarsusContactSensors[ std::make_pair( LegPos(i), j ) ] ){
						tarsusContactSensors[ std::make_pair( LegPos(i), j ) ]->sense( globalData );
					}
				}
			}
		}
    }

    void DungBot::create( const Matrix& pose )
    {
    	assert( !created );

    	odeHandle.createNewSimpleSpace(parentspace, false);

    	std::cout << "HEAD MASS:\t" << conf.massHead << std::endl;
    	std::cout << "FRONT MASS:\t" << conf.massFront << std::endl;
    	std::cout << "REAR MASS:\t" << conf.massRear << std::endl;
    	std::cout << "COXA MASS:\t" << conf.coxaMass[0] << " " << conf.coxaMass[1] << " " << conf.coxaMass[2] << " " << std::endl;
    	std::cout << "FEMUR MASS:\t" << conf.femurMass[0] << " " << conf.femurMass[1] << " " << conf.femurMass[2] << " " << std::endl;
    	std::cout << "TIBIA MASS:\t" << conf.tibiaMass[0] << " " << conf.tibiaMass[1] << " " << conf.tibiaMass[2] << " " << std::endl;
    	std::cout << "TARSUS MASS:\t" << conf.tarsusMass << std::endl;

    	/************************************
         * BODY PARTS
         ***********************************/
		//	First we find the pose for the center of the body-part to be made, then we run the functions that creates it.
    	osg::Matrix rearPos = osg::Matrix::translate( ( conf.rearDimension[0] / 2 ), 0, 0) * pose;
    	auto rear = makeBody( rearPos, conf.massRear, conf.rearDimension );

    	osg::Matrix frontPos = osg::Matrix::translate( ( -conf.frontDimension[0] / 2 ), 0, (-conf.rearDimension[2]/2.4 +conf.frontDimension[2]/2)) * pose;
		auto front = makeBody( frontPos, conf.massFront, conf.frontDimension );

		osg::Matrix headPos = osg::Matrix::translate( ( -conf.frontDimension[0] - conf.headDimension[0]/2 ), 0, (-conf.rearDimension[2]/2.4+conf.headDimension[2]/2)) * pose;
		auto head = makeHead( headPos, conf.massHead, conf.headDimension );

		//	Representation of the origin
		const Pos nullpos(0,0,0);

		//	Place the joint between the two body-parts
		makeBodyHingeJoint( front, rear, nullpos*osg::Matrix::translate( conf.frontDimension[0] / 2, 0, 0 ) * frontPos, Axis( 0, 1, 0 ) * frontPos, conf.rearDimension[1] );
		makeHeadHingeJoint( front, head, nullpos*osg::Matrix::translate( -conf.frontDimension[0] / 2, 0, 0 ) * frontPos, Axis( 0, 1, 0 ) * headPos, conf.headDimension[1] );

		/************************************
		 * 	Make position sensors
		 ***********************************/
		RelativePositionSensor rearBodySensor( 1, 1, Sensor::XYZ, false );
		bodyPartSensors.push_back( rearBodySensor );
		bodyPartSensors.back().init(rear);
		RelativePositionSensor frontBodySensor( 1, 1, Sensor::XYZ, false );
		bodyPartSensors.push_back( frontBodySensor );
		bodyPartSensors.back().init(front);
		RelativePositionSensor headBodySensor( 1, 1, Sensor::XYZ, false );
		bodyPartSensors.push_back( headBodySensor );
		bodyPartSensors.back().init(head);

		/************************************
	     * Make all the legs
	     ***********************************/
		makeAllLegs( pose , rear, front , head);

		/************************************
		 * Tilt sensor
		 ***********************************/
		OrientationSensor = new AxisOrientationSensor(AxisOrientationSensor::Axis,Sensor::X |Sensor::Y | Sensor::Z);
		OrientationSensor->init(front);

		/************************************
		 * 	Set all the parameters
		 ***********************************/
		setParam( "dummy", 0 ); // apply all parameters.

		created = true;
    }

    void DungBot::makeAllLegs( const Matrix& pose, Primitive* rear, Primitive* front, Primitive* head)
    {
    	//	Representation of the origin
    	const Pos nullpos(0,0,0);
		double xPosition=0, yPosition=0, zPosition=0;
		std::map<LegPos, osg::Matrix> legTrunkConnections;

		//	The purpose of this for-loop is to get all the leg-trunk-connections
		for (int i = 0; i < LEG_POS_MAX; i++) // Run through all of the legs
		{
			LegPos leg = LegPos(i);	// Give a value to leg (0-6)

			// Make the right legs have a negative sign
			const double lr = (leg == L0 || leg == L1 || leg == L2) - (leg == R0 || leg == R1 || leg == R2);

			// Create 3d-coordinates for the leg-body connection:
			double tempScale = conf.scale;
			switch (i)
			{
				case L0:
				case R0:
					xPosition = conf.coxaRadius[1]-2.8689/tempScale;
					yPosition = lr * 2.5409/tempScale;
					zPosition = -conf.rearDimension[2]/2 + 0.4841/tempScale;
					break;
				case L1:
				case R1:
					xPosition = conf.coxaRadius[1]+0/tempScale;
					yPosition = lr * 2.5883/tempScale;
					zPosition = -conf.rearDimension[2]/2 + 0.5672/tempScale;
					break;
				case L2:
				case R2:
					xPosition = conf.coxaRadius[1]+3.1666/tempScale;
					yPosition = lr * 4.7496/tempScale;
					zPosition = -conf.rearDimension[2]/2 + 0/tempScale;
					break;

				default:
					xPosition = 0;
			}

			Pos pos = Pos( xPosition, yPosition, zPosition );
			legTrunkConnections[leg] = osg::Matrix::translate(pos) * pose; // save the leg trunk connection for later use
		}

		std::vector<Primitive*> tarsusParts;

	    //	Create the legs
	    for (int i = 0; i < LEG_POS_MAX; i++)
	    {
			LegPos leg = LegPos(i);

			const double frontLegOnly = ( leg == R0 || leg == L0 );
			const double middleLegOnly = ( leg == R1 || leg == L1 );
			const double hindLegOnly = ( leg == R2 || leg == L2 );
			const double backLeg = ( leg == R1 || leg == R2 ) - ( leg == L1 || leg == L2 );
	        const double frontLeg = ( leg == R0 ) - ( leg == L0 );
	        const double lr = (leg == L0 || leg == L1 || leg == L2) - (leg == R0 || leg == R1 || leg == R2);

	    	/************************************
	         * Coxa Placement
	         ***********************************/
	        osg::Matrix c1 = osg::Matrix::rotate( M_PI/180*(180+95.7143), lr*hindLegOnly, 0 , 0 ) *
								osg::Matrix::rotate( M_PI/180*(180+110.4237), lr*middleLegOnly, 0 , 0 ) *
								osg::Matrix::rotate( M_PI/180*(180+112.5464), lr*frontLegOnly, 0 , 0 ) *
								osg::Matrix::rotate( M_PI/180*(90-64.6293), 0, 0 , lr*hindLegOnly ) *
								osg::Matrix::rotate( M_PI/180*(90-56.2446), 0, 0 , lr*middleLegOnly ) *
								osg::Matrix::rotate( M_PI/180*(90-65.3676), 0, 0 , lr*frontLegOnly ) *
	        					legTrunkConnections[leg];
			osg::Matrix coxaCenter = osg::Matrix::translate( 0, 0, -conf.coxaLength[i%3]/2 ) * c1; //Position of Center of Mass
			Primitive* coxaThorax = new Capsule( conf.coxaRadius[i%3], conf.coxaLength[i%3] );
			OsgHandle osgHandleCoxa = osgHandle.changeColor(255,255,0,1);
			coxaThorax->init( odeHandle, conf.coxaMass[i%3], osgHandleCoxa );
			coxaThorax->setPose( coxaCenter );
			Substance surface_coxa(100,0,25,0);
			//coxaThorax->setSubstance( surface_coxa );
			legs[leg].coxa = coxaThorax;
			objects.push_back( coxaThorax );

	    	/************************************
	         * Femur placement
	         ***********************************/
			osg::Matrix c2 = osg::Matrix::rotate( M_PI/2, 0, 0, lr ) *
								osg::Matrix::rotate( M_PI/180*120, lr, 0, 0 ) *
								osg::Matrix::translate( 0, 0, -conf.coxaLength[i%3]/2 ) *
								coxaCenter;
			osg::Matrix femurCenter = osg::Matrix::translate( 0, 0, -conf.femurLength[i%3] / 2 ) * c2;
			Primitive* femurThorax = new Capsule( conf.femurRadius[i%3], conf.femurLength[i%3]  );
			OsgHandle osgHandleFemur = osgHandle.changeColor(0,191,255,1);
			femurThorax->init( odeHandle, conf.femurMass[i%3], osgHandleFemur );
			femurThorax->setPose( femurCenter );
			Substance surface_femur(100,0,25,0);
			//femurThorax->setSubstance( surface_femur );
			legs[leg].femur = femurThorax;
			odeHandle.addIgnoredPair(femurThorax, front);
			odeHandle.addIgnoredPair(femurThorax, rear);
			odeHandle.addIgnoredPair(femurThorax, front);
			objects.push_back( femurThorax );

	    	/************************************
	         * Tibia placement
	         ***********************************/
			osg::Matrix c3 = osg::Matrix::rotate( M_PI/180*85, 0, 1, 0 ) *
								osg::Matrix::translate( 0, 0, -conf.femurLength[i%3] / 2 ) *
								femurCenter;
			osg::Matrix tibiaCenter = osg::Matrix::translate( 0, 0, -conf.tibiaLength[i%3] / 2 ) * c3;
			Primitive* tibia = new Capsule( conf.tibiaRadius[i%3], conf.tibiaLength[i%3] );
			OsgHandle osgHandleTibia = osgHandle.changeColor(247,0,255,1);
			tibia->init( odeHandle, conf.tibiaMass[i%3], osgHandleTibia );
			tibia->setPose( tibiaCenter );
			Substance surface_tibia(100,0,25,0);
			//tibia->setSubstance( surface_tibia );
			legs[leg].tibia = tibia;
			odeHandle.addIgnoredPair(tibia, front);
			odeHandle.addIgnoredPair(tibia, rear);
			odeHandle.addIgnoredPair(tibia, front);
			odeHandle.addIgnoredPair(tibia, coxaThorax);
			objects.push_back( tibia );

				/************************************
				 *	Contact sensors for femur and tibia
				 ***********************************/
				if( conf.testFemurTibiaSensor )
				{
					femurContactSensors[ std::make_pair( LegPos(i), 0) ] = new ContactSensor(true, 65, 2.5 * conf.femurRadius[i%3], false, true, Color(255,0,0));
					femurContactSensors[ std::make_pair( LegPos(i), 0) ]->setInitData(odeHandle, osgHandle, osg::Matrix::translate(0, 0, -0.5 *conf.femurLength[i%3] / 2));
					femurContactSensors[ std::make_pair( LegPos(i), 0) ]->init(femurThorax);


					tibiaContactSensors[ std::make_pair( LegPos(i), 0) ] = new ContactSensor(true, 65, 2.5 * conf.tibiaRadius[i%3], false, true, Color(255,0,0));
					tibiaContactSensors[ std::make_pair( LegPos(i), 0) ]->setInitData(odeHandle, osgHandle, osg::Matrix::translate(0, 0, -0.5 *conf.tibiaLength[i%3] / 2));
					tibiaContactSensors[ std::make_pair( LegPos(i), 0) ]->init(tibia);
				}


	    	/********************￼
￼
￼
￼Google-søgning￼Jeg prøver lykken
	    	 * ************************
	         * Calculate anchor and axis for the joints.
	         ********************************************/
			const osg::Vec3 anchor1 = nullpos * c1;
	        Axis axis1 = Axis( 0, 0, backLeg+frontLeg )*c1;

			const osg::Vec3 anchor2 = nullpos * c2;
			Axis axis2 = Axis( 0, 1, 0 )*c2;

			const osg::Vec3 anchor3 = nullpos * c3;
	        Axis axis3 = Axis( 0, 1, 0 )*c3;

	    	/********************************************
	         * BC-Joint placement (hinge-joint)
	         ********************************************/
	        if( conf.testCoxa || conf.testNo )
	        {
	        	HingeJoint* j = new HingeJoint( (leg == L0 || leg == R0) ? front : rear, coxaThorax, anchor1, -axis1 ); // Only L0 and R0 should be attached to front
				j->init( odeHandle, osgHandle.changeColor("joint"), true, conf.coxaRadius[i%3] * 3.1 );
				legs[leg].tcJoint = j;
				joints.push_back( j );
				OneAxisServo * coxaMotor = new OneAxisServoPosForce(odeHandle, j, -1.0, 1.0, 7000, 0.002, 0.7, 20.0, 1.0, true);
				legs[leg].tcServo = coxaMotor;
				servos[ getMotorName( leg, TC ) ] = coxaMotor;
	        }
	        else
	        {
	        	FixedJoint* j = new FixedJoint( (leg == L0 || leg == R0) ? front : rear, coxaThorax, anchor1 ); // Only L0 and R0 should be attached to front
	        	j->init( odeHandle, osgHandle.changeColor("joint"), true, conf.coxaRadius[i%3] * 3.1 );
				joints.push_back( j );
	        }

	    	/********************************************
	         * CF-Joint placement (hinge-joint)
	         ********************************************/
	        if( conf.testFemur || conf.testNo )
	        {
	        	HingeJoint* k = new HingeJoint( coxaThorax, femurThorax, anchor2, -axis2 );
				k->init( odeHandle, osgHandle.changeColor("joint"), true, conf.femurRadius[i%3] * 3.1 );
				legs[leg].ctJoint = k;
				joints.push_back( k );
				OneAxisServo * femurMotor = new OneAxisServoPosForce(odeHandle, k, -1.0, 1.0, 7000, 0.002, 0.7, 20.0, 1.0, true);
				legs[leg].ctrServo = femurMotor;
				servos[ getMotorName( leg, CTR ) ] = femurMotor;
	        }
	        else
	        {
	        	FixedJoint* k = new FixedJoint( coxaThorax, femurThorax, anchor2 );
				k->init( odeHandle, osgHandle.changeColor("joint"), true, conf.femurRadius[i%3] * 3.1 );
				joints.push_back( k );
	        }

	    	/********************************************
	         * FT-Joint placement (hinge-joint)
	         ********************************************/
	        if( conf.testTibia || conf.testNo )
	        {
	        	HingeJoint* l = new HingeJoint( tibia, femurThorax, anchor3, -axis3 );
				l->init( odeHandle, osgHandle.changeColor("joint"), true, conf.femurRadius[i%3] * 3.1 );
				legs[leg].ftJoint = l;
				joints.push_back( l );
				OneAxisServo * tibiaMotor = new OneAxisServoPosForce(odeHandle, l, -1.0, 1.0, 7000, 0.002, 0.7, 20.0, 1.0, true);
				legs[leg].ftiServo = tibiaMotor;
				servos[ getMotorName( leg, FTI ) ] = tibiaMotor;
	        }
	        else
	        {
	        	FixedJoint* l = new FixedJoint( femurThorax, tibia, anchor3 );
	        	l->init( odeHandle, osgHandle.changeColor("joint"), true, conf.femurRadius[i%3] * 3.1 );
	        	joints.push_back( l );
	        }

	    	/********************************************
	         * Tarsus placement
	         ********************************************/
			osg::Matrix c4 = osg::Matrix::translate( 0, 0, -conf.tibiaLength[i%3] / 2  ) *
								tibiaCenter;
			osg::Matrix tarsusCenter = osg::Matrix::translate( 0, 0, -(conf.tarsusLength[i%3] / 5)/2 ) * c4;
			Primitive *tarsus = new Capsule( conf.tarsusRadius[i%3], conf.tarsusLength[i%3] / 5 );
			OsgHandle osgHandleTarsus = osgHandle.changeColor(0,0,255,1);
			tarsus->init( odeHandle, conf.tarsusMass, osgHandleTarsus );
			tarsus->setPose( tarsusCenter );
			Substance surface_tarsus(25,0.5,50,0);
			//tarsus->setSubstance( surface_tarsus );
			legs[leg].tarsus = tarsus;
			objects.push_back( tarsus );
			tarsusParts.push_back( tarsus );

			const osg::Vec3 anchor4 = nullpos * c4;

	    	/********************************************
	         * TT-Joint placement (fixed-joint)
	         ********************************************/
			FixedJoint* q = new FixedJoint( tarsus, tibia, anchor4 );
			q->init( odeHandle, osgHandle.changeColor("joint"), true, conf.tarsusRadius[i%3] * 2.1 );
			joints.push_back(q);

			// Contact sensor
			if( conf.testTarsusSensor )
			{
				tarsusContactSensors[ std::make_pair( LegPos(i), 0) ] = new ContactSensor(true, 65, 2.5 * conf.tarsusRadius[i%3], false, true, Color(0,255,0));
				tarsusContactSensors[ std::make_pair( LegPos(i), 0) ]->setInitData(odeHandle, osgHandle, osg::Matrix::translate(0, 0, -0.5 *conf.tarsusLength[i%3] / 5));
				tarsusContactSensors[ std::make_pair( LegPos(i), 0) ]->init(tarsusParts.at(0));
			}

	    	/********************************************
	         * The five tartsus parts
	         ********************************************/
			if( conf.testTarsus )
			{
				double angle = M_PI/16;
				double radius = conf.tarsusRadius[i%3]/2;
				double partLength = conf.tarsusLength[i%3]/10;
				double mass = conf.tarsusMass/5;

				std::cout << "leg number     " << i << std::endl;

				Primitive *section = tarsus;
				osg::Matrix m6 = osg::Matrix::translate(0,0,-(conf.tarsusLength[i%3]/5)/2.4)*tarsusCenter; //

				for( int j = 1; j < 6; j++ )
				{
					 section = new Capsule( radius, partLength );
					 section->init( odeHandle, mass, osgHandleTarsus );

					 m6 = osg::Matrix::rotate(i%2==0 ? angle : -angle,0,i%2==0 ? -1 : 1,0) *
							 osg::Matrix::translate(0,0,-partLength) *
							 m6;

					 section->setPose( m6 );
					 objects.push_back( section );
					 tarsusParts.push_back( section );

					 HingeJoint* k = new HingeJoint( tarsusParts[j-1], tarsusParts[j], Pos(0,0,partLength/3) * m6, Axis(0,0,-1) * m6 );
					 k->init( odeHandle, osgHandle, true, partLength/5 * 2.1 );
					 joints.push_back( k );

					 //	Servo used as a spring
					 auto servo = std::make_shared<OneAxisServoVelocityControlled>( odeHandle, k, -1.0, 1.0, 10.0, 0.05, 20.0, 1.3 );
					 auto spring = std::make_shared<ConstantMotor>( servo, 0.0 );
					 tarsussprings.push_back( servo );
					 addMotor( spring );

					 //Contact sensor
					 if( conf.testTarsusSensor )
					 {
						 tarsusContactSensors[ std::make_pair( LegPos(i), j) ] = new ContactSensor(true, 1, 1.5 * radius, false, true, Color(0,255,0));
						 tarsusContactSensors[ std::make_pair( LegPos(i), j) ]->setInitData(odeHandle, osgHandle, osg::Matrix::translate(0, 0, -(0.5) * partLength));
						 tarsusContactSensors[ std::make_pair( LegPos(i), j) ]->init(tarsusParts.at(j));
					 }
				}
			}
			tarsusParts.clear();

	    	/********************************************
	         * Making the position sensors for each leg.
	         ********************************************/
			RelativePositionSensor tarsusSensor( 1, 1, Sensor::XYZ, false );
			bodyPartSensors.push_back( tarsusSensor );
			bodyPartSensors.back().init(tarsus);
			RelativePositionSensor tibiaSensor( 1, 1, Sensor::XYZ, false );
			bodyPartSensors.push_back( tibiaSensor );
			bodyPartSensors.back().init(tibia);
			RelativePositionSensor femurSensor( 1, 1, Sensor::XYZ, false );
			bodyPartSensors.push_back( femurSensor );
			bodyPartSensors.back().init(femurThorax);
			RelativePositionSensor coxaSensor( 1, 1, Sensor::XYZ, false );
			bodyPartSensors.push_back( coxaSensor );
			bodyPartSensors.back().init(coxaThorax);
	    }
    }

    lpzrobots::Primitive* DungBot::makeHead( const osg::Matrix& pose, const double mass, const std::vector<double> dimension )
    {
    	// Allocate object
    	lpzrobots::Primitive* head = new Box( dimension[0], dimension[1], dimension[2] );
    	// Set texture from Image library
    	head->setTexture( "../../../../../pmanoonpong-lpzrobots-fork/ode_robots/osg/data/Images/body.jpg" );
    	// Initialize the primitive
    	head->init( odeHandle, mass, osgHandle );
    	// Set pose
    	head->setPose( pose );
    	// Set substance
		//Substance surface_head(25,5,50,0);
		//head->setSubstance( surface_head );
    	// Add to objects
    	objects.push_back( head );

    	return head;
    }

    lpzrobots::Primitive* DungBot::makeBody( const Matrix& pose, const double mass, const std::vector<double> dimension )
    {
        // Allocate object
        auto bodyPart = new Box( dimension[0], dimension[1], dimension[2] );
        // Set texture from Image library
        bodyPart->setTexture( "../../../../../pmanoonpong-lpzrobots-fork/ode_robots/osg/data/Images/body.jpg");
        // Initialize the primitive
        bodyPart->init( odeHandle, mass, osgHandle );
        // Set pose
        bodyPart->setPose( pose );
        // Add to objects
        objects.push_back( bodyPart );

        return bodyPart;
    }

    void DungBot::makeBodyHingeJoint( Primitive* frontLimb, Primitive* rearLimb, const Pos position, Axis axis, const double Y )
    {
    	if( conf.testBody || conf.testNo )
    	{
	    	/********************************************
	         * Hinge joint
	         ********************************************/
    		HingeJoint* hinge = new HingeJoint( frontLimb, rearLimb, position, axis );
			hinge->init( odeHandle, osgHandle, false, Y * 1.05 );
			joints.push_back( hinge );
			OneAxisServo * bodyMotor = new OneAxisServo( hinge, -1.0, 1.0, 1.0, 0.2, 1, 10.0, 1.3, true );
			servos[DungBotMotorSensor::BJ_m] = bodyMotor;
			backboneServo = bodyMotor;
    	}
    	else
    	{
	    	/********************************************
	         * Fixed joint
	         ********************************************/
    		FixedJoint* hinge = new FixedJoint( frontLimb, rearLimb, position );
    		hinge->init( odeHandle, osgHandle, false, Y * 1.05 );
			joints.push_back( hinge );
    	}
    }

    void DungBot::makeHeadHingeJoint( Primitive* frontLimb, Primitive* rearLimb, const Pos position, Axis axis, const double Y )
	{
    	if( conf.testHead || conf.testNo )
		{
	    	/********************************************
	         * Hinge joint
	         ********************************************/
			HingeJoint* hinge = new HingeJoint( frontLimb, rearLimb, position, axis );
			hinge->init( odeHandle, osgHandle, true, Y * 1.05 );
			joints.push_back( hinge );
			OneAxisServo * headMotor = new OneAxisServo( hinge, -1.0, 1.0, 1.0, 0.2, 1, 10.0, 1.3, true );
			servos[DungBotMotorSensor::HJ_m] = headMotor;
			headServo = headMotor;
		}
		else
		{
	    	/********************************************
	         * Fixed joint
	         ********************************************/
			FixedJoint* hinge = new FixedJoint( frontLimb, rearLimb, position );
			hinge->init( odeHandle, osgHandle, false, Y * 1.05 );
			joints.push_back( hinge );
		}
	}

	void DungBot::nameMotor( const int motorNumber, const char* name )
	{
		addInspectableDescription( "y[" + std::itos( motorNumber ) + "]", name );
	}
	void DungBot::nameSensor( const int sensorNumber, const char* name )
	{
		addInspectableDescription( "x[" + std::itos( sensorNumber ) + "]", name );
	}

	void DungBot::setMotorsIntern( const double* motors, int motorNumber )
	{
		assert( created );
		assert( motorNumber >= getMotorNumberIntern() );
		for( MotorMap::iterator it = servos.begin(); it != servos.end(); it++ )
		{
			MotorName const name = it->first;
			OneAxisServo * const servo = it->second;
			if( servo )
			{
				servo->set( -motors[ name ] );
			}
		}
	}

	int DungBot::getSensorsIntern( double* sensors, int sensorNumber )
	{
		assert( created );
		assert( sensorNumber >= getSensorNumberIntern() );

		/********************************************
		 * 	Angle sensors
		 ********************************************/
		sensors[DungBotMotorSensor::TR0_as] = servos[DungBotMotorSensor::TR0_m] ? -servos[DungBotMotorSensor::TR0_m]->get() : 0;
		sensors[DungBotMotorSensor::TR1_as] = servos[DungBotMotorSensor::TR1_m] ? -servos[DungBotMotorSensor::TR1_m]->get() : 0;
		sensors[DungBotMotorSensor::TR2_as] = servos[DungBotMotorSensor::TR2_m] ? -servos[DungBotMotorSensor::TR2_m]->get() : 0;
		sensors[DungBotMotorSensor::TL0_as] = servos[DungBotMotorSensor::TL0_m] ? -servos[DungBotMotorSensor::TL0_m]->get() : 0;
		sensors[DungBotMotorSensor::TL1_as] = servos[DungBotMotorSensor::TL1_m] ? -servos[DungBotMotorSensor::TL1_m]->get() : 0;
		sensors[DungBotMotorSensor::TL2_as] = servos[DungBotMotorSensor::TL2_m] ? -servos[DungBotMotorSensor::TL2_m]->get() : 0;
		sensors[DungBotMotorSensor::CR0_as] = servos[DungBotMotorSensor::CR0_m] ? -servos[DungBotMotorSensor::CR0_m]->get() : 0;
		sensors[DungBotMotorSensor::CR1_as] = servos[DungBotMotorSensor::CR1_m] ? -servos[DungBotMotorSensor::CR1_m]->get() : 0;
		sensors[DungBotMotorSensor::CR2_as] = servos[DungBotMotorSensor::CR2_m] ? -servos[DungBotMotorSensor::CR2_m]->get() : 0;
		sensors[DungBotMotorSensor::CL0_as] = servos[DungBotMotorSensor::CL0_m] ? -servos[DungBotMotorSensor::CL0_m]->get() : 0;
		sensors[DungBotMotorSensor::CL1_as] = servos[DungBotMotorSensor::CL1_m] ? -servos[DungBotMotorSensor::CL1_m]->get() : 0;
		sensors[DungBotMotorSensor::CL2_as] = servos[DungBotMotorSensor::CL2_m] ? -servos[DungBotMotorSensor::CL2_m]->get() : 0;
		sensors[DungBotMotorSensor::FR0_as] = servos[DungBotMotorSensor::FR0_m] ? -servos[DungBotMotorSensor::FR0_m]->get() : 0;
		sensors[DungBotMotorSensor::FR1_as] = servos[DungBotMotorSensor::FR1_m] ? -servos[DungBotMotorSensor::FR1_m]->get() : 0;
		sensors[DungBotMotorSensor::FR2_as] = servos[DungBotMotorSensor::FR2_m] ? -servos[DungBotMotorSensor::FR2_m]->get() : 0;
		sensors[DungBotMotorSensor::FL0_as] = servos[DungBotMotorSensor::FL0_m] ? -servos[DungBotMotorSensor::FL0_m]->get() : 0;
		sensors[DungBotMotorSensor::FL1_as] = servos[DungBotMotorSensor::FL1_m] ? -servos[DungBotMotorSensor::FL1_m]->get() : 0;
		sensors[DungBotMotorSensor::FL2_as] = servos[DungBotMotorSensor::FL2_m] ? -servos[DungBotMotorSensor::FL2_m]->get() : 0;
		sensors[DungBotMotorSensor::BJ_as] = servos[DungBotMotorSensor::BJ_m] ? -servos[DungBotMotorSensor::BJ_m]->get() : 0;
		sensors[DungBotMotorSensor::HJ_as] = servos[DungBotMotorSensor::HJ_m] ? -servos[DungBotMotorSensor::HJ_m]->get() : 0;

		/********************************************
		 * Contact sensors, tarsus
		 ********************************************/
		if( conf.testTarsusSensor && conf.testTarsus)
		{
			sensors[DungBotMotorSensor::L0_s1] = tarsusContactSensors[std::make_pair(L0,1)]->get();
			sensors[DungBotMotorSensor::L0_s2] = tarsusContactSensors[std::make_pair(L0,2)]->get();
			sensors[DungBotMotorSensor::L0_s3] = tarsusContactSensors[std::make_pair(L0,3)]->get();
			sensors[DungBotMotorSensor::L0_s4] = tarsusContactSensors[std::make_pair(L0,4)]->get();
			sensors[DungBotMotorSensor::L0_s5] = tarsusContactSensors[std::make_pair(L0,5)]->get();

			sensors[DungBotMotorSensor::R0_s1] = tarsusContactSensors[std::make_pair(R0,1)]->get();
			sensors[DungBotMotorSensor::R0_s2] = tarsusContactSensors[std::make_pair(R0,2)]->get();
			sensors[DungBotMotorSensor::R0_s3] = tarsusContactSensors[std::make_pair(R0,3)]->get();
			sensors[DungBotMotorSensor::R0_s4] = tarsusContactSensors[std::make_pair(R0,4)]->get();
			sensors[DungBotMotorSensor::R0_s5] = tarsusContactSensors[std::make_pair(R0,5)]->get();

			sensors[DungBotMotorSensor::L1_s1] = tarsusContactSensors[std::make_pair(L1,1)]->get();
			sensors[DungBotMotorSensor::L1_s2] = tarsusContactSensors[std::make_pair(L1,2)]->get();
			sensors[DungBotMotorSensor::L1_s3] = tarsusContactSensors[std::make_pair(L1,3)]->get();
			sensors[DungBotMotorSensor::L1_s4] = tarsusContactSensors[std::make_pair(L1,4)]->get();
			sensors[DungBotMotorSensor::L1_s5] = tarsusContactSensors[std::make_pair(L1,5)]->get();

			sensors[DungBotMotorSensor::R1_s1] = tarsusContactSensors[std::make_pair(R1,1)]->get();
			sensors[DungBotMotorSensor::R1_s2] = tarsusContactSensors[std::make_pair(R1,2)]->get();
			sensors[DungBotMotorSensor::R1_s3] = tarsusContactSensors[std::make_pair(R1,3)]->get();
			sensors[DungBotMotorSensor::R1_s4] = tarsusContactSensors[std::make_pair(R1,4)]->get();
			sensors[DungBotMotorSensor::R1_s5] = tarsusContactSensors[std::make_pair(R1,5)]->get();

			sensors[DungBotMotorSensor::L2_s1] = tarsusContactSensors[std::make_pair(L2,1)]->get();
			sensors[DungBotMotorSensor::L2_s2] = tarsusContactSensors[std::make_pair(L2,2)]->get();
			sensors[DungBotMotorSensor::L2_s3] = tarsusContactSensors[std::make_pair(L2,3)]->get();
			sensors[DungBotMotorSensor::L2_s4] = tarsusContactSensors[std::make_pair(L2,4)]->get();
			sensors[DungBotMotorSensor::L2_s5] = tarsusContactSensors[std::make_pair(L2,5)]->get();

			sensors[DungBotMotorSensor::R2_s1] = tarsusContactSensors[std::make_pair(R2,1)]->get();
			sensors[DungBotMotorSensor::R2_s2] = tarsusContactSensors[std::make_pair(R2,2)]->get();
			sensors[DungBotMotorSensor::R2_s3] = tarsusContactSensors[std::make_pair(R2,3)]->get();
			sensors[DungBotMotorSensor::R2_s4] = tarsusContactSensors[std::make_pair(R2,4)]->get();
			sensors[DungBotMotorSensor::R2_s5] = tarsusContactSensors[std::make_pair(R2,5)]->get();

			sensors[DungBotMotorSensor::R0_s0] = tarsusContactSensors[std::make_pair(R0,0)]->get();
			sensors[DungBotMotorSensor::L0_s0] = tarsusContactSensors[std::make_pair(L0,0)]->get();
			sensors[DungBotMotorSensor::R1_s0] = tarsusContactSensors[std::make_pair(R1,0)]->get();
			sensors[DungBotMotorSensor::L1_s0] = tarsusContactSensors[std::make_pair(L1,0)]->get();
			sensors[DungBotMotorSensor::R2_s0] = tarsusContactSensors[std::make_pair(R2,0)]->get();
			sensors[DungBotMotorSensor::L2_s0] = tarsusContactSensors[std::make_pair(L2,0)]->get();
		} else if(conf.testTarsusSensor && !conf.testTarsus)
		{
			sensors[DungBotMotorSensor::R0_s0] = tarsusContactSensors[std::make_pair(R0,0)]->get();
			sensors[DungBotMotorSensor::L0_s0] = tarsusContactSensors[std::make_pair(L0,0)]->get();
			sensors[DungBotMotorSensor::R1_s0] = tarsusContactSensors[std::make_pair(R1,0)]->get();
			sensors[DungBotMotorSensor::L1_s0] = tarsusContactSensors[std::make_pair(L1,0)]->get();
			sensors[DungBotMotorSensor::R2_s0] = tarsusContactSensors[std::make_pair(R2,0)]->get();
			sensors[DungBotMotorSensor::L2_s0] = tarsusContactSensors[std::make_pair(L2,0)]->get();
		}

    	/********************************************
         * Position sensors
         ********************************************/
		if( true )
		{
			//	Make for loop that makes all these
			for( int i = DungBotMotorSensor::RPS_REARx; i < DungBotMotorSensor::DUNGBOT_SENSOR_MAX; i++ ){
				sensors[i] = 0;
			}

			//Rear body
			std::vector<RelativePositionSensor>::iterator it = bodyPartSensors.begin(); //we only use one goal sensor
			std::list<sensor> gls_val = it->getList();
			sensors[DungBotMotorSensor::RPS_REARx] = gls_val.back(); gls_val.pop_back();
			sensors[DungBotMotorSensor::RPS_REARy] = gls_val.back(); gls_val.pop_back();
			sensors[DungBotMotorSensor::RPS_REARz] = gls_val.back(); gls_val.pop_back();

			//Front body
			it++;	gls_val = it->getList();
			sensors[DungBotMotorSensor::RPS_FRONTx] = gls_val.back(); gls_val.pop_back();
			sensors[DungBotMotorSensor::RPS_FRONTy] = gls_val.back(); gls_val.pop_back();
			sensors[DungBotMotorSensor::RPS_FRONTz] = gls_val.back(); gls_val.pop_back();

			//Head
			it++;	gls_val = it->getList();
			sensors[DungBotMotorSensor::RPS_HEADx] = gls_val.back(); gls_val.pop_back();
			sensors[DungBotMotorSensor::RPS_HEADy] = gls_val.back(); gls_val.pop_back();
			sensors[DungBotMotorSensor::RPS_HEADz] = gls_val.back(); gls_val.pop_back();

			//	Leg1
				//	Coxa
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg1Cx] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg1Cy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg1Cz] = gls_val.back(); gls_val.pop_back();
				//	Femur
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg1Fx] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg1Fy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg1Fz] = gls_val.back(); gls_val.pop_back();
				//	Tibia
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg1Tix] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg1Tiy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg1Tiz] = gls_val.back(); gls_val.pop_back();
				//	Tarsus
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg1Tax] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg1Tay] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg1Taz] = gls_val.back(); gls_val.pop_back();
			//	Leg2
				//	Coxa
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg2Cx] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg2Cy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg2Cz] = gls_val.back(); gls_val.pop_back();
				//	Femur
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg2Fx] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg2Fy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg2Fz] = gls_val.back(); gls_val.pop_back();
				//	Tibia
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg2Tix] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg2Tiy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg2Tiz] = gls_val.back(); gls_val.pop_back();
				//	Tarsus
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg2Tax] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg2Tay] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg2Taz] = gls_val.back(); gls_val.pop_back();
			//	Leg3
				//	Coxa
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg3Cx] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg3Cy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg3Cz] = gls_val.back(); gls_val.pop_back();
				//	Femur
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg3Fx] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg3Fy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg3Fz] = gls_val.back(); gls_val.pop_back();
				//	Tibia
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg3Tix] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg3Tiy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg3Tiz] = gls_val.back(); gls_val.pop_back();
				//	Tarsus
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg3Tax] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg3Tay] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg3Taz] = gls_val.back(); gls_val.pop_back();
			//	Leg4
					//	Coxa
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg4Cx] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg4Cy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg4Cz] = gls_val.back(); gls_val.pop_back();
				//	Femur
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg4Fx] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg4Fy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg4Fz] = gls_val.back(); gls_val.pop_back();
				//	Tibia
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg4Tix] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg4Tiy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg4Tiz] = gls_val.back(); gls_val.pop_back();
				//	Tarsus
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg4Tax] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg4Tay] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg4Taz] = gls_val.back(); gls_val.pop_back();
			//	Leg5
				//	Coxa
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg5Cx] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg5Cy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg5Cz] = gls_val.back(); gls_val.pop_back();
				//	Femur
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg5Fx] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg5Fy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg5Fz] = gls_val.back(); gls_val.pop_back();
				//	Tibia
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg5Tix] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg5Tiy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg5Tiz] = gls_val.back(); gls_val.pop_back();
				//	Tarsus
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg5Tax] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg5Tay] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg5Taz] = gls_val.back(); gls_val.pop_back();
			//	Leg6
				//	Coxa
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg6Cx] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg6Cy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg6Cz] = gls_val.back(); gls_val.pop_back();
				//	Femur
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg6Fx] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg6Fy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg6Fz] = gls_val.back(); gls_val.pop_back();
				//	Tibia
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg6Tix] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg6Tiy] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg6Tiz] = gls_val.back(); gls_val.pop_back();
				//	Tarsus
					it++;	gls_val = it->getList();
					sensors[DungBotMotorSensor::RPS_Leg6Tax] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg6Tay] = gls_val.back(); gls_val.pop_back();
					sensors[DungBotMotorSensor::RPS_Leg6Taz] = gls_val.back(); gls_val.pop_back();

		}

		/********************************************
		 * Contact sensors, femur and tibia
		 ********************************************/
		if( conf.testFemurTibiaSensor )
		{
			sensors[DungBotMotorSensor::L0_CSF] = femurContactSensors[std::make_pair(L0,0)]->get();
			sensors[DungBotMotorSensor::L1_CSF] = femurContactSensors[std::make_pair(L1,0)]->get();
			sensors[DungBotMotorSensor::L2_CSF] = femurContactSensors[std::make_pair(L2,0)]->get();
			sensors[DungBotMotorSensor::R0_CSF] = femurContactSensors[std::make_pair(R0,0)]->get();
			sensors[DungBotMotorSensor::R1_CSF] = femurContactSensors[std::make_pair(R1,0)]->get();
			sensors[DungBotMotorSensor::R2_CSF] = femurContactSensors[std::make_pair(R2,0)]->get();

			sensors[DungBotMotorSensor::L0_CST] = tibiaContactSensors[std::make_pair(L0,0)]->get();
			sensors[DungBotMotorSensor::L1_CST] = tibiaContactSensors[std::make_pair(L1,0)]->get();
			sensors[DungBotMotorSensor::L2_CST] = tibiaContactSensors[std::make_pair(L2,0)]->get();
			sensors[DungBotMotorSensor::R0_CST] = tibiaContactSensors[std::make_pair(R0,0)]->get();
			sensors[DungBotMotorSensor::R1_CST] = tibiaContactSensors[std::make_pair(R1,0)]->get();
			sensors[DungBotMotorSensor::R2_CST] = tibiaContactSensors[std::make_pair(R2,0)]->get();

		}

		/********************************************
		 *	Tilt sensor
		 ********************************************/
		std::list<sensor> Ori_lst =  OrientationSensor->getList();
		double tmp;
		tmp = Ori_lst.front(); Ori_lst.pop_front();
		sensors[DungBotMotorSensor::BX_ori] = tmp;
		tmp = Ori_lst.front(); Ori_lst.pop_front();
		sensors[DungBotMotorSensor::BY_ori] = tmp;
		tmp = Ori_lst.front();
		sensors[DungBotMotorSensor::BY_ori] = tmp;

		return DungBotMotorSensor::DUNGBOT_SENSOR_MAX;
	}

	int DungBot::getMotorNumberIntern( void )
	{
		return DungBotMotorSensor::DUNGBOT_MOTOR_MAX;
	}

	int DungBot::getSensorNumberIntern( void )
	{
		return DungBotMotorSensor::DUNGBOT_SENSOR_MAX;
	}

	DungBot::MotorName DungBot::getMotorName( LegPos leg, LegJointType joint )
	{
		if (leg == L0 && joint == TC)	return DungBotMotorSensor::TL0_m;
		if (leg == L0 && joint == CTR)	return DungBotMotorSensor::CL0_m;
		if (leg == L0 && joint == FTI)	return DungBotMotorSensor::FL0_m;
		if (leg == L1 && joint == TC)	return DungBotMotorSensor::TL1_m;
		if (leg == L1 && joint == CTR)	return DungBotMotorSensor::CL1_m;
		if (leg == L1 && joint == FTI)	return DungBotMotorSensor::FL1_m;
		if (leg == L2 && joint == TC)	return DungBotMotorSensor::TL2_m;
		if (leg == L2 && joint == CTR)	return DungBotMotorSensor::CL2_m;
		if (leg == L2 && joint == FTI)	return DungBotMotorSensor::FL2_m;
		if (leg == R0 && joint == TC)	return DungBotMotorSensor::TR0_m;
		if (leg == R0 && joint == CTR)	return DungBotMotorSensor::CR0_m;
		if (leg == R0 && joint == FTI)	return DungBotMotorSensor::FR0_m;
		if (leg == R1 && joint == TC)	return DungBotMotorSensor::TR1_m;
		if (leg == R1 && joint == CTR)	return DungBotMotorSensor::CR1_m;
		if (leg == R1 && joint == FTI)	return DungBotMotorSensor::FR1_m;
		if (leg == R2 && joint == TC)	return DungBotMotorSensor::TR2_m;
		if (leg == R2 && joint == CTR)	return DungBotMotorSensor::CR2_m;
		if (leg == R2 && joint == FTI)	return DungBotMotorSensor::FR2_m;
		return DungBotMotorSensor::DUNGBOT_MOTOR_MAX;
	}

	bool DungBot::setParam( const paramkey& key, paramval val )
	{
	    bool rv = Configurable::setParam(key, val);
	    int i = 0;
	    for( LegMap::iterator it = legs.begin(); it != legs.end(); it++ )
	    {
	    	//std::cout << "test my nigger" << it << std::endl;
			Spring * const tarsusSpring = it->second.tarsusSpring;
			if( tarsusSpring )
			{
				tarsusSpring->setPower( conf.tarsus_Kp );
				tarsusSpring->setDamping( conf.tarsus_Kd );
				tarsusSpring->setIntegration( conf.tarsus_Ki );
				tarsusSpring->setPower( conf.tarsusMaxVel );
			}

			OneAxisServo * tc = it->second.tcServo;
			if( tc )
			{
				tc->setPower( conf.coxa_Kp[i%3] );
				tc->setDamping( conf.coxa_Kd[i%3] );
				tc->setIntegration( conf.coxa_Ki[i%3] );
				tc->setMaxVel( conf.coxaMaxVel[i%3] );			// Power scale for the motor

				if (it->first == L2 || it->first == R2) tc->setMinMax(conf.rCoxaJointLimitF, conf.rCoxaJointLimitB);
				if (it->first == L1 || it->first == R1) tc->setMinMax(conf.mCoxaJointLimitF, conf.mCoxaJointLimitB);
				if (it->first == L0 || it->first == R0) tc->setMinMax(conf.fCoxaJointLimitF, conf.fCoxaJointLimitB);
			}

			OneAxisServo * ctr = it->second.ctrServo;
			if(ctr)
			{
				ctr->setPower( conf.femur_Kp[i%3] );
				ctr->setDamping( conf.femur_Kd[i%3] );
				ctr->setIntegration( conf.femur_Ki[i%3] );
				ctr->setMaxVel( conf.femurMaxVel[i%3] );   	// Power scale for the motor

				//	Min is up, up is negative
				if (it->first == L2 || it->first == R2) ctr->setMinMax(conf.rFemurJointLimitU, conf.rFemurJointLimitD);
				if (it->first == L1 || it->first == R1) ctr->setMinMax(conf.rFemurJointLimitU, conf.mFemurJointLimitD);
				if (it->first == L0 || it->first == R0) ctr->setMinMax(conf.rFemurJointLimitU, conf.fFemurJointLimitD);
			}

			OneAxisServo * fti = it->second.ftiServo;
			if( fti )
			{
				fti->setPower( conf.tibia_Kp[i%3] );
				fti->setDamping( conf.tibia_Kd[i%3] );
				fti->setIntegration( conf.tibia_Ki[i%3] );
				fti->setMaxVel( conf.tibiaMaxVel[i%3] ); 	// Power scale for the motor

				if (it->first == L2 || it->first == R2) fti->setMinMax(conf.rTibiaJointLimitU, conf.rTibiaJointLimitD);
				if (it->first == L1 || it->first == R1) fti->setMinMax(conf.mTibiaJointLimitU, conf.mTibiaJointLimitD);
				if (it->first == L0 || it->first == R0) fti->setMinMax(conf.fTibiaJointLimitU, conf.fTibiaJointLimitD);
			}
			i++;
		}

		if( backboneServo && (conf.testBody || conf.testNo ) )
		{
			backboneServo->setPower( conf.back_Kp );
			backboneServo->setDamping( conf.back_Kd );
			backboneServo->setIntegration( conf.back_Ki );

			backboneServo->setMaxVel( conf.backMaxVel );
			backboneServo->setMinMax( conf.backJointLimitU, conf.backJointLimitD );
		}

		if( headServo && (conf.testHead || conf.testNo) )
		{
			headServo->setPower( conf.head_Kp );
			headServo->setDamping( conf.head_Kd );
			headServo->setIntegration( conf.head_Ki );

			headServo->setMaxVel( conf.headMaxVel );
			headServo->setMinMax( conf.headJointLimitU, conf.headJointLimitD );
		}

		return rv;
	}

	DungBotConf DungBot::getDefaultConf( void )
	{
		DungBotConf conf;

    	/********************************************
         * What to test?
         ********************************************/
		conf.testNo = false;				//	If true, then all hinges exist.
		conf.testHead = false;				//	If true, then Head hinges is made else fixed joints.
		conf.testBody = false;				//	If true, then Body hinges is made else fixed joints.
		conf.testCoxa = true;				//	If true, then Coxa hinges is made else fixed joints.
		conf.testFemur = true;				//	If true, then Femur hinges is made else fixed joints.
		conf.testTibia = true;				//	If true, then Tibia hinges is made else fixed joints.

		conf.testFemurTibiaSensor = true;	//	If true, then contact sensors on femur and tibia is created.

		conf.testTarsus = false; 			// 	If true, then tarsus is created, else it is not created
		conf.testTarsusSensor = true;

    	/********************************************
         * Body dimensions
         ********************************************/
		double totalLength = 3.75+9.111+10.324;
		conf.scale = totalLength;

		conf.headDimension 	= { 3.75/totalLength, 4.568/totalLength, 2.9/totalLength};
		conf.frontDimension = { 5.146/totalLength, 9.111/totalLength, 4.3/totalLength };
		conf.rearDimension 	= { 9.028/totalLength, 10.324/totalLength, 4.9/totalLength };

		double totalMass = 106.402/10;
		conf.massHead = 14.826/totalMass;
		conf.massFront = 23.823/totalMass;
		conf.massRear = 30.439/totalMass;

    	/********************************************
         * Leg dimensions
         ********************************************/
		//	Coxa
		conf.coxaLength = {
							2.46037/totalLength,
							2.11888/totalLength,
							4.05514/totalLength };
		conf.coxaRadius = {
							1.65197/totalLength/4,
							1.65227/totalLength/4,
							1.63748/totalLength/4 };
		conf.coxaMass = {
							1.2979/totalMass,
							1.5078/totalMass,
							3.0317/totalMass };
		//	Femur
		conf.femurLength = {
							3.24472/totalLength,
							4.23025/totalLength,
							4.63394/totalLength };
		conf.femurRadius = {
							1.876084/totalLength/4,
							2.00444/totalLength/4,
							2.41763/totalLength/4 };
		conf.femurMass = {
							2.8817/totalMass,
							2.2400/totalMass,
							2.6258/totalMass };
		//	Tibia
		conf.tibiaLength = {
							4.68943/totalLength,
							3.72093/totalLength,
							5.54793/totalLength };
		conf.tibiaRadius = {
							1.04005/totalLength/4,
							0.917478/totalLength/4,
							0.924187/totalLength/4 };
		conf.tibiaMass = {
							1.5269/totalMass,
							1.3660/totalMass,
							2.1793/totalMass };

		std::cout << "Total mass: " << 14.826/totalMass+23.823/totalMass+30.439/totalMass+2*(1.2979/totalMass+1.5078/totalMass+3.0317/totalMass+2.8817/totalMass+2.2400/totalMass+2.6258/totalMass+1.5269/totalMass+1.3660/totalMass+2.1793/totalMass) << std::endl;

		//	Tarsus
		conf.tarsusLength ={
							3.4765/totalLength,
							3.00413/totalLength,
							3.94887/totalLength };
		conf.tarsusRadius = {
							0.250162/totalLength/2,
							0.247163/totalLength/2,
							0.25316/totalLength/2 };
		conf.tarsusMass = 0.01;	//TODO: Find a proper mass for the tarsus


    	/********************************************
         * Range of motion for the joints (D=DOWN U=UP)
         ********************************************/
		conf.backJointLimitD = M_PI / 180 * 0.0;
		conf.backJointLimitU =	-M_PI / 180 * 45.0;

		conf.headJointLimitD = M_PI / 180 * 45.0;
		conf.headJointLimitU =	-M_PI / 180 * 0.0;

		double rotationScale = 0.9;

		double coxaFront = 95.7878*rotationScale;
		double coxaMiddle = 116.1153*rotationScale;
		double coxaHind = 160.8514*rotationScale;
		double A = 80;
		double B = 50;
		double C = 50;

		//	TC JOINT
		conf.fCoxaJointLimitF = -M_PI / 180.0 * A;				// 70 deg; forward (-) MAX --> normal walking range 60 deg MAX
		conf.fCoxaJointLimitB =  M_PI / 180.0 * (coxaFront-A);	//-70 deg; backward (+) MIN --> normal walking range -10 deg MIN
	    conf.mCoxaJointLimitF = -M_PI / 180.0 * B;				// 60 deg; forward (-) MAX --> normal walking range 30 deg MAX
	    conf.mCoxaJointLimitB =  M_PI / 180.0 * (coxaMiddle-B);	// 60 deg; backward (+) MIN --> normal walking range -40 deg MIN
	    conf.rCoxaJointLimitF = -M_PI / 180.0 * C;				// 70 deg; forward (-) MAX --> normal walking range 60 deg MAX
	    conf.rCoxaJointLimitB =  M_PI / 180.0 * (coxaHind-C);	// 70 deg; backward (+) MIN --> normal walking range -10 deg MIN

	    double femur = 90*rotationScale;
	    A = 30;
	    B = 30;
	    C = 30;

	    //	CT JOIN
	    conf.fFemurJointLimitD =  M_PI / 180.0 * A;
	    conf.fFemurJointLimitU = -M_PI / 180.0 * ( femur-A );
	    conf.mFemurJointLimitD =  M_PI / 180.0 * B;
	    conf.mFemurJointLimitU = -M_PI / 180.0 * ( femur-B );
	    conf.rFemurJointLimitD =  M_PI / 180.0 * C;
	    conf.rFemurJointLimitU = -M_PI / 180.0 * ( femur-C );

	    double tibia = 170*rotationScale;
		A = 85;
		B = 85;
		C = 85;

	    //	FT JOINT
	    conf.fTibiaJointLimitD =  M_PI / 180.0 * A;
	    conf.fTibiaJointLimitU = -M_PI / 180.0 * ( tibia-A );
	    conf.mTibiaJointLimitD =  M_PI / 180.0 * B;
	    conf.mTibiaJointLimitU = -M_PI / 180.0 * ( tibia-B );
	    conf.rTibiaJointLimitD =  M_PI / 180.0 * C;
	    conf.rTibiaJointLimitU = -M_PI / 180.0 * ( tibia-C );

    	/********************************************
         * PID VALUES
         ********************************************/
	    conf.head_Kp	= 1.5;
	    conf.back_Kp 	= 4.0;
		conf.coxa_Kp 	= {4.3, 5.0, 4.8}; 	// Originally 2.5 *This new speed (and 2.5) works very well, but when changed, the legs gets out of sync
		conf.femur_Kp	= {4.3, 5.0, 4.8}; 	// Originally 2.5
		conf.tibia_Kp 	= {4.3, 5.0, 4.8};	// Originally 2.5
		conf.tarsus_Kp 	= 0.0;

		conf.head_Kd	= 0.0;
		conf.back_Kd 	= 0.0;
		conf.coxa_Kd 	= {0.5, 0.5, 0.5};
		conf.femur_Kd 	= {0.5, 0.5, 0.5};
		conf.tibia_Kd 	= {0.5, 0.5, 0.5};
		conf.tarsus_Kd	= 0.0;

		conf.head_Ki	= 0.5;
		conf.back_Ki 	= 0.0;
		conf.coxa_Ki 	= {0.75, 0.75, 0.75};
		conf.femur_Ki 	= {0.75, 0.75, 0.75};
		conf.tibia_Ki 	= {0.75, 0.75, 0.75};
		conf.tarsus_Ki	= 0.0;

		conf.backMaxVel 	= 2.0;
		conf.coxaMaxVel 	= {2.0, 2.0, 2.0};
		conf.femurMaxVel 	= {2.0, 2.0, 2.0};
		conf.tibiaMaxVel 	= {2.0, 2.0, 2.0};

		return conf;
	}
}
