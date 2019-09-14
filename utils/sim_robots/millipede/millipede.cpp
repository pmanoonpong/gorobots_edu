/***************************************************************************
 *   Copyright (C) 2012 by                                                 *
 *    Martin Biehl <mab@physik3.gwdg.de>                                   *
 *    Guillaume de Chambrier <s0672742@sms.ed.ac.uk>                       *
 *    martius@informatik.uni-leipzig.de                                    *
 *    Timo Nachstedt <nachstedt@physik3.gwdg.de>                           *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 **************************************************************************/

//#define VERBOSE


// include header file
#include "millipede.h"

// rotation and translation matrixes (to make the code shorter)
#define ROTM osg::Matrix::rotate
#define TRANSM osg::Matrix::translate

#define ERROR std::cout << "passed over here.. \n";

namespace lpzrobots {

  Millipede::Leg::Leg() {
    tcJoint = 0;
    ctrJoint = 0;
    ftiJoint = 0;
    footJoint = 0;
    tcServo = 0;
    ctrServo = 0;
    ftiServo = 0;
    footSpring = 0;
    shoulder = 0;
    coxa = 0;
    second = 0;
    tibia = 0;
    foot = 0;
  }

  // constructor:
  // - give handle for ODE and OSG stuff
  // also initialize millipede.conf with the configuration in the argument of
  // the constructor
  Millipede::Millipede(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const MillipedeConf& c, const std::string& name) :
          OdeRobot(odeHandle, osgHandle, name, "millipede 0.0"), conf(c) {

    // robot is not created till now
    created = false;
//    backboneServo = 0;
    usSensorFrontLeft = 0;
    usSensorFrontRight = 0;
    speedsensor = 0;

    NumberOfLegs = 0;
    NumberOfMotors = 0;
    NumberOfSensors = 0;

    addParameter("coxaPower", &conf.coxaPower);
    addParameter("secondPower", &conf.secondPower);
    addParameter("coxaDamp", &conf.coxaDamping);
    addParameter("fcoxaJointLimitF", &conf.fcoxaJointLimitF);
    addParameter("fcoxaJointLimitB", &conf.fcoxaJointLimitB);
    addParameter("mcoxaJointLimitF", &conf.mcoxaJointLimitF);
    addParameter("mcoxaJointLimitB", &conf.mcoxaJointLimitB);
    addParameter("rcoxaJointLimitF", &conf.rcoxaJointLimitF);
    addParameter("rcoxaJointLimitB", &conf.rcoxaJointLimitB);
    addParameter("secondJointLimitD", &conf.secondJointLimitD);
    addParameter("secondJointLimitU", &conf.secondJointLimitU);
    addParameter("coxaMaxVel", &conf.coxaMaxVel);

    if (conf.useTebiaJoints) {
      addParameter("tebiaPower", &conf.tebiaPower);
      addParameter("tebiaDamp", &conf.tebiaDamping);
      addParameter("tebiaJointLimitD", &conf.tebiaJointLimitD);
      addParameter("tebiaJointLimitU", &conf.tebiaJointLimitU);
    }


    // add further inspectables
//    addInspectableValue("posX", &position.x, "x Position of robot");
//    addInspectableValue("posY", &position.y, "y Position of robot");
//    addInspectableValue("posZ", &position.z, "z Position of robot");

  }

  Millipede::~Millipede() {
    destroy();
  }

  int Millipede::getMotorNumberIntern() {
    return servos.size()+doubleServos.size()*2;
  }


  /**
   * Assign a human readable name to a sensor. This name is used for the
   * associated inspectable value as used e.g. in guilogger.
   *
   * @param motorNo index of the motor (for standard motors defined by
   *        the SensorName enum)
   * @param name human readable name for the sensor
   */
  void Millipede::nameSensor(const int sensorNo, const char* name) {
#ifdef VERBOSE
    std::cerr << "millipede::nameSensor BEGIN\n";
#endif
    addInspectableDescription("x[" + std::itos(sensorNo) + "]", name);
#ifdef VERBOSE
    std::cerr << "millipede::nameSensor END\n";
#endif
  }

  /**
   * Assign a human readable name to a motor. This name is used for the
   * associated inspectable value as used e.g. in guilogger.
   *
   * @param motorNo index of the motor (for standard motors defined by
   *        the MotorName enum)
   * @param name human readable name for the motor
   */
  void Millipede::nameMotor(const int motorNo, const char* name) {
#ifdef VERBOSE
    std::cerr << "millipede::nameMotor BEGIN\n";
#endif
    addInspectableDescription("y[" + std::itos(motorNo) + "]", name);
#ifdef VERBOSE
    std::cerr << "millipede::nameMotor END\n";
#endif
  }

  /* sets actual motorcommands
   @param motors motors scaled to [-1,1]
   @param motornumber length of the motor array
   */
  void Millipede::setMotorsIntern(const double* motors, int motornumber) {
#ifdef VERBOSE
    std::cerr << "millipede::setMotors BEGIN\n";
#endif
    assert(created);
    // robot must exist
    assert(motornumber==getMotorNumber());

    //leg motor commands
    for (MotorMap::iterator it = servos.begin(); it != servos.end(); it++) {
      int const name = it->first;
      OneAxisServo * const servo = it->second;
      //We multiple with -1 to map to real hexapod
      if (servo){
          servo->set(-motors[name]);
      }
    }

    //intersegment motor commands
    for (LinkMap::iterator it = doubleServos.begin(); it != doubleServos.end(); it++){
        int const name = it->first;
        TwoAxisServo *const servo = it->second;
        if (servo){
            servo->set(-motors[servos.size()+name*2], -motors[servos.size()+name*2+1]);
        }
    }

#ifdef VERBOSE
    std::cerr << "millipede::setMotors END\n";
#endif
  }


  int Millipede::getSensorNumberIntern() {
#ifdef VERBOSE
    std::cerr << "millipede::getSensorNumberIntern BEGIN\n";
#endif
#ifdef VERBOSE
    std::cerr << "millipede::getSensorNumberIntern END\n";
#endif
    return NumberOfSensors;
  }


  /* returns actual sensorvalues
   @param sensors sensors scaled to [-1,1] (more or less)
   @param sensornumber length of the sensor array
   @return number of actually written sensors
   */
  int Millipede::getSensorsIntern(sensor* sensors, int sensornumber) {
#ifdef VERBOSE
    std::cerr << "millipede::getSensors BEGIN\n";
#endif
    assert(created);
    assert(sensornumber >= getSensorNumberIntern());
#ifdef VERBOSE
    std::cerr << "millipede::getSensors END\n";
#endif
//    std::cout << "sensornumber: " << sensornumber << std::endl;

    for(int i = 0; i<NumberOfLegs; i++)
        sensors[i] = 0;

    if (conf.legContactSensorIsBinary) { // No scaling since binary signals are already in the range of [0,..,1]

        for(int i = 0; i < NumberOfLegs; i++){
            sensors[i] = legContactSensors[i] ? legContactSensors[i]->get() : 0;
        }

    } else { // Scaling since analog signals are used then we scale them to the range of [0,..,1]
            //To be checked for different body weights!! Noise can make the value go above 1 or below 0

        for(unsigned int i = 0; i < legContactSensors.size(); i++)
            sensors[i] = legContactSensors[i] ? ((legContactSensors[i]->get())/1.5) : 0;
    }

    sensors[legContactSensors.size()] = position.x;
    sensors[legContactSensors.size()+1] = position.y;
    sensors[legContactSensors.size()+2] = position.z;

    return NumberOfSensors;
  }


  void Millipede::placeIntern(const osg::Matrix& pose) {
#ifdef VERBOSE
    std::cerr << "millipede::place BEGIN\n";
#endif
    // the position of the robot is the center of the first segment
    // to set the vehicle on the ground when the z component of the position
    // is 0
    //Matrix p2 = pose * ROTM(0, 0, conf.legLength + conf.legLength/8);
    osg::Matrix p = pose
        * TRANSM(0, 0, conf.tebiaLength - conf.shoulderHeight + 2 * conf.tebiaRadius + conf.footRadius);
    create(p);
#ifdef VERBOSE
    std::cerr << "millipede::place END\n";
#endif
  }


  /**
   * updates the osg notes
   */
  void Millipede::update() {
    OdeRobot::update();
#ifdef VERBOSE
    std::cerr << "millipede::update BEGIN\n";
#endif
    assert(created);
    // robot must exist

    for (PrimitiveList::iterator i = objects.begin(); i != objects.end(); i++) {
      if (*i)
        (*i)->update();
    }
    for (JointList::iterator i = joints.begin(); i != joints.end(); i++) {
      if (*i)
        (*i)->update();
    }
    for (JointList2::iterator i = joints2.begin(); i != joints2.end(); i++) {
        if (*i)
            (*i)->update();
    }
    // update the graphical representation of the sensorbank
    if(conf.irSensors)
        irSensorBank->update();

    for (int i = 0; i < NumberOfLegs; i++) {
      if (legContactSensors[i])
        legContactSensors[i]->update();
    }

#ifdef VERBOSE
    std::cerr << "millipede::update END\n";
#endif
  }


  double Millipede::getMassOfRobot() {

    double totalMass = 0.0;

    for (unsigned int i = 0; i < objects.size(); i++) {
      dMass massOfobject;
      dBodyGetMass(objects[i]->getBody(), &massOfobject);
      totalMass += massOfobject.mass;
    }
    return totalMass;
  }

  void Millipede::sense(GlobalData& globalData) {
    OdeRobot::sense(globalData);
    if(conf.irSensors)
        irSensorBank->sense(globalData);

    for (int i = 0; i < NumberOfLegs; i++) {
      if (legContactSensors[i])
        legContactSensors[i]->sense(globalData);
    }
  }

  /**
   * this function is called in each timestep. It should perform robot-
   * internal checks, like space-internal collision detection, sensor
   * resets/update etc.
   *
   * @param global structure that contains global data from the simulation
   * environment
   */
  void Millipede::doInternalStuff(GlobalData& global) {

#ifdef VERBOSE
    std::cerr << "millipede::doInternalStuff BEGIN\n";
#endif

    // update statistics
    position = getPosition();

    // passive servos have to be set to zero in every time step so they work
    // as springs
    for (ServoList::iterator it = passiveServos.begin(); it != passiveServos.end(); it++) {
      (*it)->set(0.0);
    }

#ifdef VERBOSE
    std::cerr << "millipede::doInternalStuff END\n";
#endif
  }

  Primitive* Millipede::getMainPrimitive() const {
    return segments[0].primitive;
  }

  /**
   * creates vehicle at desired position
   *
   * @param pos struct Position with desired position
   */
  void Millipede::create(const osg::Matrix& pose) {
#ifdef VERBOSE
    std::cerr << "millipede::create BEGIN\n";
#endif
    if (created) {
      destroy();
    }

    // we want legs colliding with other legs, so we set internal collision
    // flag to "false".
    odeHandle.createNewSimpleSpace(parentspace, false);

    // color of robot
    osgHandle = osgHandle.changeColor("robot1");

    // color of joint axis
    OsgHandle osgHandleJoint = osgHandle.changeColor("joint");

//    // change Material substance
    OdeHandle odeHandleBody = odeHandle;
    odeHandleBody.substance.toMetal(2.0);


    /**********************************************************************/
    /*  create body                                                       */
    /**********************************************************************/
    /** central position of the trunk */
    const osg::Matrix trunkPos = pose;

    //segment trunk
    millipedeSegment seg;
    seg.height = conf.height;
    seg.width = conf.width;
    seg.length = conf.size - conf.frontLength;
    seg.nOfLegs = conf.legsPerSegment;
    seg.position = trunkPos;

    if(conf.nOfSegments == 1){
        //create segment trunk
        seg.position = TRANSM(0, 0, 0) * trunkPos;
        createSegment(seg, &odeHandleBody);


        /************************************
         * adding LEGS
         ***********************************/
        //currently, only four leg set up implemented

        for(int i = 0; i < seg.nOfLegs; i++){
            int leg = i;
            double legPositionX;
            if(i > 1)
                legPositionX = conf.legpos1-0.075;
            else
                legPositionX = conf.legpos2+0.075;

            createLeg(leg, legPositionX, 0, &osgHandleJoint);

        }
    }else{
        for(int j = 0; j < conf.nOfSegments; j++){

            //create segment trunk
            seg.position = TRANSM(-(seg.length+conf.linkLength*2)*j, 0, 0) * trunkPos;
            createSegment(seg, &odeHandleBody);


            /************************************
             * adding LEGS
             ***********************************/
            //currently, only four leg set up implemented
            if(seg.nOfLegs == 2){
                for(int i = 0; i < seg.nOfLegs; i++){
                    int leg = i;
                    double legPositionX;
                    legPositionX = 0;
                    createLeg(leg, legPositionX, j, &osgHandleJoint);

                }
            }else{

                for(int i = 0; i < seg.nOfLegs; i++){
                    int leg = i;
                    double legPositionX;
                    if(i > 1)
                        legPositionX = conf.legpos1;
                    else
                        legPositionX = conf.legpos2;

                    createLeg(leg, legPositionX, j, &osgHandleJoint);

                }
            }

            //adding universal joint between segment trunks
            if(j>0)
                createLink(j, &osgHandleJoint);
        }
    }

    //adding IR sensor at front
    if(conf.irSensors){
        // initialize the infrared sensors
        irSensorBank = new RaySensorBank();
        irSensorBank->setInitData(odeHandle, osgHandle, TRANSM(0,0,0));
        irSensorBank->init(0);

        // ultrasonic sensors at Front part
        usSensorFrontRight = new IRSensor();
        irSensorBank->registerSensor(usSensorFrontRight, segments[0].primitive,
          ROTM(M_PI / 2, conf.usAngleX, conf.usAngleY, 0)
          * TRANSM(0.3 * (conf.useBack ? conf.frontLength : conf.size), -0.25 * conf.width, -0.45 * conf.height),
          conf.usRangeFront, RaySensor::drawRay);

        NumberOfSensors++;
    }

    // sensors for position
    NumberOfSensors+=3;

    setParam("dummy", 0); // apply all parameters.

    created = true;
#ifdef VERBOSE
    std::cerr << "millipede::create END\n";
#endif

}



///////////////////////////////////////////////////////////
/// \brief Millipede::createLeg
/// \param leg: leg index
/// \param legPositionX: position on segment
/// \param segmentIndex: segment where attached
/// \param osgHandleJoint: osgHandle
///
void Millipede::createLeg(int leg, double legPositionX, int segmentIndex, OsgHandle *osgHandleJoint){

    const osg::Matrix& segmentPose = segments[segmentIndex].position;

    osg::Matrix legtrunkconnections;
    osg::Matrix shouldertrunkconnections;

    double pos_lr = (leg%2) *2-1;

    const double l0 = conf.shoulderLength;
    const double t0 = conf.shoulderRadius;
    const double l1 = conf.coxaLength;
    const double t1 = conf.coxaRadius;
    const double l2 = conf.secondLength;
    const double t2 = conf.secondRadius;
    const double l3 = conf.tebiaLength - 2 * conf.tebiaRadius - conf.footRange;
    const double t3 = conf.tebiaRadius;
    const double l4 = 2 * conf.tebiaRadius + conf.footRange - conf.footRadius;
    const double t4 = conf.footRadius;

    //Here the position of the legs was defined so left go with pos_lr = 1 and pos_lr = -1, different from the definition used
    //afterwards, just inverting pos_lr makes the legs be placed correctly.
    // create 3d-coordinates for the leg-trunk connection:
    Pos pos = Pos(
        // from (0,0,0) we go down x-axis, make two legs then up
        // legdist1 and so on
        legPositionX,
        // switch left or right side of trunk for each leg
        -pos_lr * conf.width / 2,
        // height of leg fixation to trunk (trunk bottom sits at
        // total legLength)
        -conf.height / 2 + 0.03);

    // get a coordinate system at the position pos by rotating such that
    // z-axis points toward trunk, pose is where the robot will be
    // placed so we begin there.
    legtrunkconnections = ROTM(M_PI / 2, -pos_lr, 0, 0) * TRANSM(pos) * segmentPose;


    shouldertrunkconnections = ROTM(M_PI / 2, -pos_lr, 0, 0) * TRANSM(pos);

    // if wanted, leg trunk connections are rotated here:
    legtrunkconnections= ROTM(conf.mLegRotAngle, 0, 0, pos_lr) * ROTM(conf.mLegTrunkAngleH, pos_lr, 0, 0)
            * ROTM(conf.mLegTrunkAngleV, 0, 1, 0) * legtrunkconnections;
    // also the relative coordinates for the shoulders
    shouldertrunkconnections = ROTM(conf.mLegRotAngle, 0, 0, pos_lr) * ROTM(conf.mLegTrunkAngleH, pos_lr, 0, 0)
            * ROTM(conf.mLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections;

    // get a representation of the origin
    const Pos nullpos(0,0,0);

    osg::Matrix c1;

    // m0 is the position where the center of mass of the zeroth limb
    // capsule is placed
    osg::Matrix m0;
    if (conf.useShoulder) {
        //shift connection of coxa outward
        c1 = TRANSM(0, 0, -l0) * legtrunkconnections;
        //create shoulder
        Primitive * should = new Capsule(t0, l0);
        should->setTexture(conf.texture);
        // add shoulder to trunk body
        // the shoulder's pose has to be given relative to the trunk's pose
        // add the first four shoulders to center the other two to front
        Primitive * trans = new Transform(segments[segmentIndex].primitive, should,
            TRANSM(0, 0, -l0 / 2) * shouldertrunkconnections);
        trans->init(odeHandle, conf.shoulderMass, osgHandle);
        legs[getLegIndex(segmentIndex,leg)].shoulder = trans;
        objects.push_back(trans);


        // m1 is the position where the center of mass of the first limb
        // capsule is placed
        osg::Matrix m1 = TRANSM(0, 0, -l1 / 2) * c1;

        // calculate anchor of the first joint
        const osg::Vec3 anchor1 = nullpos * c1;
        // and it's axis (multiplication with c1 indicates in which
        // (local) coordinate system it is)
        const Axis axis1 = Axis(0, 1, 0) * c1;

        // proceed along the leg (and the respective z-axis) for second
        // limb
        osg::Matrix c2 = TRANSM(0, 0, -l1 / 2) * m1;
        osg::Matrix m2 = TRANSM(0, 0, -l2 / 2) * c2;
        const osg::Vec3 anchor2 = nullpos * c2;
        const Axis axis2 = Axis(pos_lr, 0, 0) * c2;

        //and third
        osg::Matrix c3 = TRANSM(0, 0, -l2 / 2) * m2;
        osg::Matrix m3 = TRANSM(0, 0, -l3 / 2) * c3;
        const osg::Vec3 anchor3 = nullpos * c3;
        const Axis axis3 = Axis(pos_lr, 0, 0) * c3;
        // now create first limp
        Primitive* coxaThorax;
        // create upper limp with radius t1 and length l1 (length refers
        // only to length of the cylinder without the semispheres at
        // both ends)
        coxaThorax = new Capsule(t1, l1);
        coxaThorax->setTexture(conf.texture);
        coxaThorax->init(odeHandle, conf.coxaMass, osgHandle);
        //put it at m1
        coxaThorax->setPose(m1);
        legs[getLegIndex(segmentIndex,leg)].coxa = coxaThorax;
        objects.push_back(coxaThorax);
        if (conf.useShoulder) {
            odeHandle.addIgnoredPair(legs[getLegIndex(segmentIndex,leg)].shoulder, coxaThorax);
        }
        // powered hip joint of trunk to first limb
        HingeJoint* j = new HingeJoint(segments[segmentIndex].primitive, coxaThorax, anchor1, -axis1);
        j->init(odeHandle, *osgHandleJoint, true, t1 * 2.1);
        joints.push_back(j);
        // create motor, overwrite the jointLimit argument with 1.0
        // because it is less obscure and setMinMax makes mistakes
        // otherwise. Parameters are set later
        OneAxisServo * servo1 = new OneAxisServoVel(odeHandle, j, -1, 1, 1, 0.01, 0, 1.0);
        //PUSH THIS STUFF BACK INTO STH!!!! not hipservos obviously
        legs[getLegIndex(segmentIndex,leg)].tcServo = servo1;
        servos[getMotorIndex(segmentIndex, leg, TC)] = servo1;
        NumberOfMotors++;

        // second limb
        Primitive* secondThorax;
        secondThorax = new Capsule(t2, l2);
        secondThorax->setTexture(conf.texture);
        secondThorax->init(odeHandle, conf.secondMass, osgHandle);
        secondThorax->setPose(m2);
        legs[getLegIndex(segmentIndex,leg)].second = secondThorax;
        objects.push_back(secondThorax);

        // create the joint from first to second limb (coxa to second)
        HingeJoint* k = new HingeJoint(coxaThorax, secondThorax, anchor2, -axis2);
        k->init(odeHandle, *osgHandleJoint, true, t1 * 2.1);
        legs[getLegIndex(segmentIndex,leg)].ctrJoint = k;
        joints.push_back(k);
        /** parameters are set later */
        OneAxisServo * servo2 = new OneAxisServoVel(odeHandle, k, -1, 1, 1, 0.01, 0, 1.0);
        //PUSH THIS STUFF BACK INTO STH!!!! not hipservos obviously
        legs[getLegIndex(segmentIndex,leg)].ctrServo = servo2;
        servos[getMotorIndex(segmentIndex, leg, CTR)] = servo2;
        NumberOfMotors++;

        // third limb
        Primitive* tebia;
        tebia = new Capsule(t3, l3);
        tebia->setTexture(conf.texture);
        tebia->init(odeHandle, conf.tebiaMass, osgHandle);
        tebia->setPose(m3);
        //        tebiaPos.push_back(tebia->getPosition());
        legs[getLegIndex(segmentIndex,leg)].tibia = tebia;
        objects.push_back(tebia);

        // IR sensor at each leg
//        IRSensor* sensor = new IRSensor();
//        irSensorBank->registerSensor(sensor, tebia,
//          ROTM(M_PI / 2, 0, 1, 0) * TRANSM(1.01 * t3, 0, -0.2 * conf.tebiaLength), conf.irRangeLeg,
//          RaySensor::drawRay);
//        irLegSensors[leg] = sensor;

        // springy knee joint
        HingeJoint* l = new HingeJoint(secondThorax, tebia, anchor3, -axis3);
        l->init(odeHandle, *osgHandleJoint, true, t3 * 2.1);
        legs[getLegIndex(segmentIndex,leg)].ftiJoint = l;
        joints.push_back(l);
        // servo used as a spring
        /** parameters are set later */
        OneAxisServo * servo3 = new OneAxisServoVel(odeHandle, l, -1, 1, 1, 0.01, 0, 1.0);
        legs[getLegIndex(segmentIndex,leg)].ftiServo = servo3;
        servos[getMotorIndex(segmentIndex, leg, FTI)] = servo3;
        NumberOfMotors++;

        //spring foot at the end
        if (conf.useFoot) {
            osg::Matrix c4 = TRANSM(0, 0, -l3 / 2 - 2 * conf.tebiaRadius - conf.footRange + conf.footRadius) * m3;
            osg::Matrix m4 = TRANSM(0, 0, -conf.footSpringPreload) * c4;

            const osg::Vec3 anchor4 = nullpos * m4;
            const Axis axis4 = Axis(0, 0, -1) * c4;

            OdeHandle my_odeHandle = odeHandle;
            if (conf.rubberFeet) {
              const Substance FootSubstance(3.0, 0.0, 500.0, 0.1);
              my_odeHandle.substance = FootSubstance;
            }

            Primitive* foot;
            foot = new Capsule(t4, l4);
            foot->setTexture(conf.texture);
            foot->init(my_odeHandle, conf.footMass, osgHandle);
            foot->setPose(m4);
            //            footPos.push_back(foot->getPosition());
            legs[getLegIndex(segmentIndex,leg)].foot = foot;
            objects.push_back(foot);

            SliderJoint* m = new SliderJoint(tebia, foot, anchor4, axis4);
            m->init(odeHandle, *osgHandleJoint, true, t3, true);
            legs[getLegIndex(segmentIndex,leg)].footJoint = m;
            joints.push_back(m);

            /** parameters are set later */
            Spring* spring = new Spring(m, -1, 1, 1);
            legs[getLegIndex(segmentIndex,leg)].footSpring = spring;
            passiveServos.push_back(spring);
            odeHandle.addIgnoredPair(secondThorax, foot);

            legContactSensors[getLegIndex(segmentIndex,leg)] = new ContactSensor(conf.legContactSensorIsBinary, 50, 1.01 * t4, true);
            legContactSensors[getLegIndex(segmentIndex,leg)]->setInitData(odeHandle, osgHandle, TRANSM(0, 0, -0.5 * l4));
            legContactSensors[getLegIndex(segmentIndex,leg)]->init(foot);
            odeHandle.addIgnoredPair(tebia, legContactSensors[leg]->getTransformObject());
            }
    }

    NumberOfSensors++;
    NumberOfLegs++;

}

void Millipede::createLink(int segmentIndex, OsgHandle *osgHandleJoint){

    //get a representation of the origin
    const Pos nullpos(0, 0, 0);

    // Fixation to follower segment:
    osg::Matrix linktrunkconnections;
    Pos pos = Pos(segments[segmentIndex].length/2, 0, 0);

    osg::Matrix c1;
    linktrunkconnections = ROTM(M_PI / 2, 0, -1, 0)*TRANSM(pos) ;

    c1 = TRANSM(0, 0, -conf.linkLength) * linktrunkconnections * segments[segmentIndex].position;

//    Primitive * link2 = new Capsule(conf.linkRadius, conf.linkLength);
//    link2->setTexture(conf.texture);

//    Primitive * trans2 = new Transform(segments[segmentIndex].primitive, link2,
//        TRANSM(0, 0, -conf.linkLength / 2) * linktrunkconnections);
//    trans2->init(odeHandle, conf.shoulderMass, osgHandle);
////    legs[getLegIndex(segmentIndex,leg)].shoulder = trans;
//    objects.push_back(trans2);

    //create joints:
    osg::Matrix m1 = TRANSM(0, 0, -conf.linkLength / 2) * c1;

//    Primitive* link1;

//    link1 = new Capsule(conf.linkRadius, conf.linkLength);
//    link1->setTexture(conf.texture);
//    link1->init(odeHandle, conf.coxaMass, osgHandle);
//    //put it at m1
//    link1->setPose(m1);
//    objects.push_back(link1);

    // calculate anchor of the first joint
    const osg::Vec3 anchor1 = nullpos * c1;
    // and it's axis (multiplication with c1 indicates in which
    // (local) coordinate system it is)
    const Axis axis1 = Axis(0, 1, 0) * c1;
    const Axis axis2 = Axis(1, 0, 0) * c1;

    // powered hip joint of trunk to first limb
    UniversalJoint* j = new UniversalJoint(segments[segmentIndex].primitive, segments[segmentIndex-1].primitive, anchor1, -axis1, -axis2);
    j->init(odeHandle, *osgHandleJoint, true, conf.linkRadius * 2.1);
    joints2.push_back(j);
    // create motor, overwrite the jointLimit argument with 1.0
    // because it is less obscure and setMinMax makes mistakes
    // otherwise. Parameters are set later

    TwoAxisServo * servo1 = new TwoAxisServoVel(odeHandle, j, -0.3, 0.3, 100, -0.3, 0.3, 100, 0.1, 10.0, 0.1);

    doubleServos[getMotorIndex(segmentIndex)] = servo1;
    NumberOfMotors++;

//    FixedJoint* fixator = new  FixedJoint(segments[segmentIndex-1].primitive, link1);
//    fixator->init(odeHandle, osgHandle, false);


}

void Millipede::createSegment(millipedeSegment segment, OdeHandle *odeHandleBody){

    segment.primitive = new Box(segment.length, segment.width, segment.height);
    segment.primitive->setTexture(conf.bodyTexture);
    segment.primitive->init(*odeHandleBody, conf.frontMass, osgHandle.changeColor("robot2"));
    segment.primitive->setPose(segment.position);
    objects.push_back(segment.primitive);
    segments.push_back(segment);


}


  /** destroys vehicle and space
   */
  void Millipede::destroy() {
    if (created) {
#ifdef VERBOSE
      std::cerr << "begin millipede::destroy\n";
#endif
      // delete contact sensors
      for (int i = 0; i < NumberOfLegs; i++) {
        if (legContactSensors[i])
          delete legContactSensors[i];
      }
      legContactSensors.clear();

      // remove all ignored pairs (brute force method)
      for (PrimitiveList::iterator i = objects.begin(); i != objects.end(); i++) {
        for (PrimitiveList::iterator j = objects.begin(); j != objects.end(); j++) {
          if (odeHandle.isIgnoredPair((*i)->getGeom(), (*j)->getGeom())) {
            odeHandle.removeIgnoredPair((*i)->getGeom(), (*j)->getGeom());
          }
        }

      }
      if(conf.irSensors){
          irSensorBank->clear();
          delete irSensorBank;
      }

      if (speedsensor) {
        delete speedsensor;
        speedsensor = 0;
      }

      for (MotorMap::iterator it = servos.begin(); it != servos.end(); it++) {
        if (it->second)
          delete (it->second);
      }
      servos.clear();

      for (ServoList::iterator it = passiveServos.begin(); it != passiveServos.end(); it++) {
        if (*it)
          delete (*it);
      }
      passiveServos.clear();

      for (JointList::iterator i = joints.begin(); i != joints.end(); i++) {
        if (*i)
          delete *i;
      }
      joints.clear();

      for (JointList2::iterator i = joints2.begin(); i != joints2.end(); i++) {
        if (*i)
          delete *i;
      }
      joints2.clear();

      for (PrimitiveList::iterator i = objects.begin(); i != objects.end(); i++) {
        if (*i)
          delete *i;
      }
      objects.clear();

      //should all be empty as objects were cleared:
      legs.clear();

//      //------------------ delete GoalSensor here by Ren--------------------
//      GoalSensor.clear();
//      //------------------ delete GoalSensor here by Ren--------------------

      odeHandle.deleteSpace();
#ifdef VERBOSE
      std::cerr << "end millipede::destroy\n";
#endif
    }

    created = false;
  }

  bool Millipede::setParam(const paramkey& key, paramval val) {
#ifdef VERBOSE
    std::cerr << "millipede::setParam BEGIN\n";
#endif
    // the parameters are assigned here
    bool rv = Configurable::setParam(key, val);

    // we simply set all parameters here
    for (LegMap::iterator it = legs.begin(); it != legs.end(); it++) {
      Spring * const footspring = it->second.footSpring;
      if (footspring) {
        footspring->setPower(conf.footPower);
        footspring->setDamping(conf.footDamping);
        footspring->setMaxVel(conf.footMaxVel);
        //yes, min is up, up is negative
        footspring->setMinMax(conf.footSpringLimitD, conf.footSpringLimitU);
      }

      OneAxisServo * tc = it->second.tcServo;
      if (tc) {
        tc->setPower(conf.coxaPower);
        tc->setDamping(conf.coxaDamping);
        tc->setMaxVel(conf.coxaMaxVel);
        tc->setMinMax(conf.rcoxaJointLimitF, conf.rcoxaJointLimitB);
      }

      OneAxisServo * ctr = it->second.ctrServo;
      if (ctr) {
        ctr->setPower(conf.secondPower);
        ctr->setDamping(conf.secondDamping);
        ctr->setMaxVel(conf.secondMaxVel);
        //yes, min is up, up is negative
        ctr->setMinMax(conf.secondJointLimitU, conf.secondJointLimitD);
      }

      OneAxisServo * fti = it->second.ftiServo;
      if (fti) {
        fti->setPower(conf.tebiaPower);
        fti->setDamping(conf.tebiaDamping);
        fti->setMaxVel(conf.tebiaMaxVel);
        //yes, min is up, up is negative
        fti->setMinMax(conf.tebiaJointLimitU, conf.tebiaJointLimitD);
      }
    }

//    if (backboneServo) {
//      backboneServo->setPower(conf.backPower);
//      backboneServo->setDamping(conf.backDamping);
//      backboneServo->setMaxVel(conf.backMaxVel);
//      backboneServo->setMinMax(conf.backJointLimitU, conf.backJointLimitD);
//    }

#ifdef VERBOSE
    std::cerr << "millipede::setParam END\n";
#endif
    return rv;
  }


/// Returns the motor identification number for links between segments.
///
/// Based on number of segments, legs and motors per leg.
int Millipede::getMotorIndex(int linkNum){

      return linkNum-1;
}
/// Returns the motor identification number for leg motors.
/// Based on number of segments, legs and motors per leg.
int Millipede::getMotorIndex(int segmentNum, int legNum, int motNum){

    int totalprevLegs = 0;

    for(int i = 0; i < segmentNum; i++){
        totalprevLegs+=segments[i].nOfLegs;
    }

    int ind = totalprevLegs*3+legNum*3+motNum;
    return ind;

}

/// Returns leg identification number.
/// Based on number of segments and legs per segment.
int Millipede::getLegIndex(int segmentNum, int legNum){
    int totalprevLegs = 0;

    for(int i = 0; i < segmentNum; i++){
        totalprevLegs+=segments[i].nOfLegs;
    }
    int ind = totalprevLegs+legNum;
    return ind;
}

int Millipede::getLinkIndex(int linkNum){
    return linkNum-1;
}

  MillipedeConf Millipede::getDefaultConf(double _scale, int _legsPerSegment, int _nOfSegments, bool _useShoulder, bool _useFoot, bool _useBack) {
    return getMillipedeConf(_scale, _legsPerSegment, _nOfSegments, _useShoulder, _useFoot, _useBack);
  }

  MillipedeConf Millipede::getMillipedeConf(double _scale, int _legsPerSegment, int _nOfSegments, bool _useShoulder, bool _useFoot, bool _useBack) {

    MillipedeConf c;

    c.legsPerSegment = _legsPerSegment;
    c.nOfSegments = _nOfSegments;
    c.jointsFixed = true;

    // "Internal" variable storing the currently used version
    c.amos_version = 2;
    // use shoulder (fixed joint between legs and trunk)
    c.useShoulder = _useShoulder;
    c.useTebiaJoints = 0;
    //create springs at the end of the legs
    c.useFoot = _useFoot;
    //create a joint in the back
    c.useBack = _useBack;
    c.rubberFeet = false;
    c.useLocalVelSensor = 0;
    c.legContactSensorIsBinary = false;

    // the trunk length. this scales the whole robot! all parts' sizes,
    // masses, and forces will be adapted!!
    c.size = 0.53 * _scale;
    //trunk width
    c.width = 18.0 / 43.0 * c.size;
    //trunk height
    c.height = 20 / 43.0 * c.size;
    c.frontLength = 18.0 / 43.0 * c.size;
    // we use as density the original trunk weight divided by the original
    // volume

    //Change mass by KOH to 3.0
    const double density = 1.0 / (0.43 * 0.07 * 0.065); //2.2 / (0.43 * 0.07 * 0.065);

    c.trunkMass = density * c.size * c.width * c.height;
    // use the original trunk to total mass ratio
    const double mass = 5.758 / 2.2 * c.trunkMass;
    c.frontMass = c.trunkMass * c.frontLength / c.size;
    // distribute the rest of the weight like this for now */
    c.shoulderMass = (mass - c.trunkMass) / (6 * (3.0 + c.useShoulder)) * (20.0 - c.useFoot) / 20.0;
    c.coxaMass = c.shoulderMass;
    c.secondMass = c.shoulderMass;
    c.tebiaMass = c.shoulderMass;
    // foot gets 3 or 4 times 1/20 of shoulderMass (divide and multiply by
    // 3 or 4)
    c.footMass = (mass - c.trunkMass) / 6 * c.useFoot / 20.0;

    //As real robot!!
    const double shoulderHeight_cm = 5.5;
    //shoulder height "4.5 wrong" --> correct=6.5 cm from rotating point
    c.shoulderHeight = shoulderHeight_cm / 5.5 * c.height;

    // distance between hindlegs and middle legs
    c.legpos1 = -7.0 / 43.0 * c.size;
    // distance between middle legs and front legs
    c.legpos2 = 7.0 / 43.0 * c.size;

    // configure the wheels (if used). They don't have any counterpart in
    // reality, so the chosen values are arbitrary
    c.wheel_radius = 0.10 * c.size;
    c.wheel_width = 0.04 * c.size;
    c.wheel_mass = (mass - c.trunkMass) / 6.0;

    // -----------------------
    // 1) Biomechanics
    // Manual setting adjustable joint positions at the body
    // -----------------------

    // millipede has a fixed but adjustable joint that decides how the legs
    // extend from the trunk. Here you can adjust these joints

    // ------------- Front legs -------------
    // angle (in rad) around vertical axis at leg-trunk fixation 0:
    // perpendicular
    // => forward/backward
    c.fLegTrunkAngleV = 0.0;
    // angle around horizontal axis at leg-trunk fixation 0: perpendicular
    // => upward/downward
    c.fLegTrunkAngleH = 0.0;
    // rotation of leg around own axis 0: first joint axis is vertical
    // => till
    c.fLegRotAngle = 0.0;

    // ------------- Middle legs ----------------
    // => forward/backward
    c.mLegTrunkAngleV = 0.0;
    // => upward/downward
    c.mLegTrunkAngleH = 0.0;
    // => till
    c.mLegRotAngle = 0.0;

    // ------------- Rear legs ------------------
    // => forward/backward
    c.rLegTrunkAngleV = 0.0;
    // => upward/downward
    c.rLegTrunkAngleH = 0.0;
    // => till
    c.rLegRotAngle = 0.0;

    // be careful changing the following dimension, they may break the
    // simulation!! (they shouldn't but they do)
    const double shoulderLength_cm = 4.5;
    c.shoulderLength = shoulderLength_cm / 43.0 * c.size;
    c.shoulderRadius = .03 * c.size;

    const double coxaLength_cm = 3.5;
    c.coxaLength = coxaLength_cm / 43.0 * c.size;
    c.coxaRadius = .04 * c.size;

    const double secondLength_cm = 6.0;
    c.secondLength = secondLength_cm / 43.0 * c.size;
    c.secondRadius = .03 * c.size;
    c.tebiaRadius = 1.3 / 43.0 * c.size;

    const double tebiaLength_cm = 11.5; // 3)
    c.tebiaLength = tebiaLength_cm / 43.0 * c.size;

    const double linkLenght_cm = 2; //from similarity with shoulder
    c.linkLength = linkLenght_cm/43.0*c.size;
    c.linkRadius = 0.04 *c.size;

    // this determines the limit of the footspring
    c.footRange = .2 / 43.0 * c.size;
    c.footRadius = 1.5 / 43.0 * c.size;

    // -----------------------
    // 2) Joint Limits
    // Setting Max, Min of each joint with respect to real
    // -----------------------

    //Similar to real robot
    //-45 deg; downward (+) MIN
    c.backJointLimitD = M_PI / 180 * 45.0;
    // 45 deg; upward (-) MAX
    c.backJointLimitU = -M_PI / 180 * 45.0;

    // 70 deg; forward (-) MAX --> normal walking range 60 deg MAX
    c.fcoxaJointLimitF = -M_PI / 180.0 * 70.0;
    //-70 deg; backward (+) MIN --> normal walking range -10 deg MIN
    c.fcoxaJointLimitB = M_PI / 180.0 * 70.0;

    //60 deg; forward (-) MAX --> normal walking range 30 deg MAX
    c.mcoxaJointLimitF = -M_PI / 180.0 * 60.0;
    //60 deg; backward (+) MIN --> normal walking range -40 deg MIN
    c.mcoxaJointLimitB = M_PI / 180 * 60.0;

    //70 deg; forward (-) MAX --> normal walking range 60 deg MAX
    c.rcoxaJointLimitF = -M_PI / 180.0 * 70.0;
    //70 deg; backward (+) MIN --> normal walking range -10 deg MIN
    c.rcoxaJointLimitB = M_PI / 180.0 * 70.0;

    // 70 deg; downward (+) MIN
    c.secondJointLimitD = M_PI / 180.0 * 75.0;
    // 70 deg upward (-) MAX
    c.secondJointLimitU = -M_PI / 180.0 * 75.0;

    //130 deg downward; (+) MIN
    c.tebiaJointLimitD = M_PI / 180.0 * 130.0;
    // 20 deg  downward; (+) MAX
    c.tebiaJointLimitU = M_PI / 180.0 * 20.0;

    // -----------------------
    // 3) Motors
    // Motor power and joint stiffness
    // -----------------------

    c.footSpringPreload = 8.0 / 43.0 * c.size;
    // negative is downwards (spring extends)
    c.footSpringLimitD = c.footSpringPreload;
    c.footSpringLimitU = c.footSpringPreload + c.footRange;

    const double backPower_scale = 30.0;
    const double coxaPower_scale = 10.0;
    const double springstiffness = 350.0;

    // use an original radius and mass and scale original torque by their
    // new values to keep acceleration constant
    c.backPower = backPower_scale * (1.962 / (0.035 * 2.2)) * c.coxaLength * c.trunkMass;
    // torque in Nm
    c.coxaPower = coxaPower_scale * (1.962 / (0.035 * 2.2)) * c.coxaLength * c.trunkMass;
    c.secondPower = c.coxaPower;
    c.tebiaPower = c.coxaPower;
    // this is the spring constant. To keep  acceleration for the body
    // constant, we use the above unscaled preload of 0.08 and original
    // trunkMass to and then multiply by the new ones
    c.footPower = (springstiffness * 0.08 / 2.2) * c.trunkMass / c.footSpringPreload;

    c.backDamping = 0.0;
    // Georg: no damping required for new servos
    c.coxaDamping = 0.0;
    c.secondDamping = 0.0;
    c.tebiaDamping = 0.01;
    c.footDamping = 0.05; // a spring has no damping??

    //Increasing MaxVel by KOH to a factor of 1.7
    c.backMaxVel = 1.7 * 1.961 * M_PI;
    // The speed calculates how it works
    c.coxaMaxVel = 1.7 * 1.961 * M_PI;
    c.secondMaxVel = 1.7 * 1.961 * M_PI;
    c.tebiaMaxVel = 1.7 * 1.961 * M_PI;
    c.footMaxVel = 1.7 * 1.961 * M_PI;

    c.usRangeFront = 0.3 * c.size;
    c.irRangeLeg = 0.2 * c.size;

    //Values by Dennis
    // 1 is parallel, -1 is antiparallel
    c.usParallel = false;
    c.usAngleX = 0.5;
    c.usAngleY = 1;

    c.texture = "Images/whiteground.rgb";
    c.bodyTexture = "Images/stripes.rgb";

    //----------------Add GoalSensor by Ren------------------
    c.GoalSensor_references.clear(); //enforce empty vector -> no relative position sensing
    //----------------Add GoalSensor by Ren------------------

    // Add IR sensors:
    c.irSensors = false;

    return c;
  }


// Getting motor command address
  int motorIdentity(const MillipedeConf &conf, int segmentNum, int legNum, int motNum){

      int totalprevLegs = 0;

      for(int i = 0; i < segmentNum; i++){
          totalprevLegs+=conf.legsPerSegment;
      }

      int ind = totalprevLegs*3+legNum*3+motNum;
      return ind;

  }


// Getting sensor address
    int touchSensorIdentity(const MillipedeConf &conf, int segmentNum, int legNum){

        int totalprevLegs = 0;

        for(int i = 0; i < segmentNum; i++){
            totalprevLegs+=conf.legsPerSegment;
        }
        int ind = totalprevLegs+legNum;
        return ind;

    }
}

