/*****************************************************************************
*   "THE BEER-WARE LICENSE" (Revision 43):
*   This software was written by Theis Str�m-Hansen <thstroemhansen@gmail.com>
*   and Mathias Thor <mathias.thor@gmail.com>
*   As long as you retain this notice you can do whatever you want with it.
*   If we meet some day, and you think this stuff is worth it, you can buy me
*   a beer in return.
*
*   Should this software ever become self-aware, remember: I am your master
*****************************************************************************/

#ifndef __DUNGBOT_H
#define __DUNGBOT_H

//	include motor and sensor definitions
#include "DungBotSensorMotorDefinition.h"

// #include <ode/ode.h>
#include <ode-dbl/ode.h>

// include primitives (box, spheres, cylinders ...)
#include <ode_robots/primitive.h>

// include sensors
#include <ode_robots/contactsensor.h>
#include <ode_robots/relativepositionsensor.h>

// include joints
#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>
#include <ode_robots/constantmotor.h>
#include <ode_robots/spring.h>

// Extra includes
#include <selforg/inspectable.h>
#include <selforg/configurable.h>
#include <ode_robots/oderobot.h>
#include <vector>
#include <iostream>
#include <string>

//	Includes of typedefs and extra
#include "DungBotIncludes.h"

namespace lpzrobots
{
	class DungBotSimple : public OdeRobot, public Inspectable
	{
		DungBotConf conf;
		public:
		//	Public attributes
			enum LegPos
			{
				L0, L1, L2, R0, R1, R2, LEG_POS_MAX
			};

			enum LegJointType
			{
				//  Thoroca-Coxal joint for forward (+) and backward (-) movements.
				TC,
				//  Coxa-Trochanteral joint for elevation (+) and depression (-) of the leg
				CTR,
				//  Femur-Tibia joints for extension (+) and flexion (-) of the tibia
				FTI,
				//  Maximum value, used for iteration
				LEG_JOINT_TYPE_MAX
			};

			typedef DungBotMotorSensor::DungBotMotorNames MotorName;
			typedef DungBotMotorSensor::DungBotSensorNames SensorName;

		public:
		//	Public methods
			static DungBotConf getDefaultConf( void );

			DungBotSimple( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
						const DungBotConf &conf = getDefaultConf(),
						const std::string& name = "DungBot" );

			virtual void placeIntern( const osg::Matrix& pose ) override;
			virtual void create( const osg::Matrix& pose );
			virtual void doInternalStuff( GlobalData& globalData );
			virtual void update( void );
			virtual void sense( GlobalData& globalData );
			virtual ~DungBotSimple();

			virtual void setMotorsIntern( const double* motors, int motornumber );
			virtual int getSensorsIntern( sensor* sensors, int sensornumber );
			virtual int getSensorNumberIntern( void );
			virtual int getMotorNumberIntern( void );
			static MotorName getMotorName( LegPos leg, LegJointType joint );

			virtual bool setParam( const paramkey& key, paramval val );

		protected:
		//	Protected attributes
			Position startPosition;
			Position position;

			struct Leg
			{
				Leg();
				HingeJoint * tcJoint;
				HingeJoint * ctJoint;
				HingeJoint * ftJoint;
				/*Slider*/Joint * footJoint;
				OneAxisServo * tcServo;
				OneAxisServo * ctrServo;
				OneAxisServo * ftiServo;
				Spring * tarsusSpring;
				Primitive * shoulder;
				Primitive * coxa;
				Primitive * femur; 	//Called second in AmosII
				Primitive * tibia;
				Primitive * tarsus;
				Primitive * foot;
			};

		protected:
		//	Protected methods
			void nameMotor( const int motorNo, const char* name );
			void nameSensor( const int sensorNo, const char* name );

		private:
		//	Private attributes
			bool created;

			//	Typedefs
			typedef std::map< LegPos, Leg > LegMap;
			typedef std::map< MotorName, OneAxisServo* > MotorMap;
			typedef std::map< std::pair< LegPos, int >, ContactSensor* > TarsusContactMap;
			typedef std::vector< OneAxisServo* > ServoList;

			//	For legs
			std::vector< std::shared_ptr< OneAxisServoVelocityControlled > > tarsussprings;
			LegMap legs;

			//	For servos
			OneAxisServo * backboneServo;
			OneAxisServo * headServo;
			ServoList passiveServos;
			MotorMap servos;

			//	For sensors
			TarsusContactMap tarsusContactSensors;
			std::vector<RelativePositionSensor> bodyPartSensors;

		private:
		//	Private methods
			lpzrobots::Primitive* makeBody( const osg::Matrix&, const double , const std::vector<double> );
			lpzrobots::Primitive* makeLegPart( const osg::Matrix&, const double , const double, const double );
			lpzrobots::Primitive* makeFoot( const osg::Matrix& );
			lpzrobots::Primitive* makeHead( const osg::Matrix&, const double, const std::vector<double> );
			void makeAllLegs( const osg::Matrix& pose, Primitive*, Primitive* , Primitive*);
			void makeHeadHingeJoint( Primitive*, Primitive*, const Pos, Axis, const double );
			void makeBodyHingeJoint( Primitive*, Primitive*, const Pos, Axis, const double );
			void makeHingeJoint( Primitive*, Primitive*, const Pos, Axis, const double );
			void makeFixedJoint( Primitive*, Primitive*, const Pos, const double );

	};
} //End namespace lpzrobot

#endif
