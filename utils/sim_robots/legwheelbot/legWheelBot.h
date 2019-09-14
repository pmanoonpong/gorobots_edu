// Header guard
#ifndef __LEGWHEELBOT_H
#define __LEGWHEELBOT_H

// Include ODE Robot class to inherit from it
#include <ode_robots/oderobot.h>

// ODE primitives
#include <ode_robots/primitive.h>

// ODE joints for objects
#include <ode_robots/joint.h>

// ODE angular motors
#include <ode_robots/angularmotor.h>

//ODE axisorientation sensors
#include <ode_robots/axisorientationsensor.h>

//ODE Relative position sensor
#include <ode_robots/relativepositionsensor.h>

#include <ode_robots/rotationsensor.h>

// Using name space lpzrobots
namespace lpzrobots {

  // structures to hold configuration of the robot
  typedef struct {
    double wheelRadius;         // Radius of the cylinder defining the wheel
    double wheelMass;           // Mass of the wheel
    int noOfSpokes;
    double spokeRadius;
    double spokeLength;
    double wheelMotorPower;     // Maximum power allowed to the motor to reach MaxSpeed
    double wheelMotorMaxSpeed;  // Maximum speed of the wheel
  } LegWheelBotWheelConf;
  
  typedef struct {
    double bodyRadius;          // Radius of the cylinder defining the body
    double bodyHeight;          // Height of the cylinder defining the body
    double bodyMass;            // Mass of the body
    LegWheelBotWheelConf rightWheel;
    LegWheelBotWheelConf leftWheel;
  } LegWheelBotConf;

  /**
   * LegWheelBot robot: two separated wheel on each side of the body
   * Inherit from OdeRobot
   */
  class LegWheelBot: public OdeRobot {
    public:
      // Structure to hold the configuration of the robot
      LegWheelBotConf conf;

      /**
       * Contrustructor
       */
      LegWheelBot(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                   const LegWheelBotConf &conf = getDefaultConf(),
                   const std::string& name = "LegWheelBot");

      /**
       * Default configuration of the robot
       */
      static LegWheelBotConf getDefaultConf(){
        LegWheelBotConf conf;
        conf.bodyRadius         = 2;
        conf.bodyHeight         = .5;
        conf.bodyMass           = 1.;
        conf.rightWheel 	= getDefaultWheel();
	conf.leftWheel		= getDefaultWheel();
        return conf;
      }
      
      static LegWheelBotWheelConf getDefaultWheel() {
	LegWheelBotWheelConf wheel;
	wheel.wheelRadius       = 0.1;
        wheel.wheelMass         = 5.;
	wheel.noOfSpokes	= 10;
	wheel.spokeLength	= 1;
	wheel.spokeRadius	= .1;
        wheel.wheelMotorPower   = 30.;
        wheel.wheelMotorMaxSpeed= 8.;
	
	return wheel;
      }
      
      static LegWheelBotConf getConf(int rNoOfSpokes, float rSpokeLength, int lNoOfSpokes, float lSpokeLength) {
	LegWheelBotConf conf = getDefaultConf();
	//Right wheel
	conf.rightWheel.noOfSpokes = rNoOfSpokes;
	conf.rightWheel.spokeLength = rSpokeLength;
	//Left wheel
	conf.leftWheel.noOfSpokes = lNoOfSpokes;
	conf.leftWheel.spokeLength = lSpokeLength;
        
        return conf;
      }

      /**
       * Destructor
       */
      virtual ~LegWheelBot();

      /**
       * Place the robot in the desired pose
       * @param pose desired 4x4 pose matrix
       */
      virtual void placeIntern(const osg::Matrix& pose) override;

      /**
       * Create the robot in the desired pose
       * @param pose desired 4x4 pose matrix
       */
      virtual void create(const osg::Matrix& pose);
      
      private:
            void attachWheel(const osg::Matrix& bodyPose, int translationDirection, Cylinder *body, LegWheelBotWheelConf wheelConf, std::string motorName);
	    void attachTail(const osg::Matrix& bodyPose, Cylinder *body, LegWheelBotConf conf);
            void createTestObject(osg::Matrix& pose);
  };
} // end namespace lpzrobots


// End of header guard
#endif
