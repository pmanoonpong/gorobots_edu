// Header Guard
#ifndef __TRIBOT_H
#define __TRIBOT_H

#include <iostream>
#include <string>
// Include ODE robot to inherit
#include <ode_robots/oderobot.h>
// ODE primitive
#include <ode_robots/primitive.h>
// ODE joints
#include <ode_robots/joint.h>
// Angular Motor
#include <ode_robots/angularmotor.h>

// Using namespace lpzrobots
namespace lpzrobots {
  /**
   * Tribot inspired by https://www.adafruit.com/products/3244
   * A robot with 2 wheels, one on each side, and a small tap in the front to keep balance
   */
  class Tribot : public OdeRobot {
  public:


    typedef struct {
      double bodyRadius;          // Radius of the cylinder defining the body
      double bodyHeight;          // Height of the cylinder defining the body
      double bodyMass;            // Mass of the body
      double wheelRadius;         // Radius of the cylinder defining the wheel
      double wheelHeight;         // Height of the cylinder defining the wheel
      double wheelMass;           // Mass of the wheel
      double wheelMotorPower;     // Maximum power allowed to the motor to reach MaxSpeed
      double wheelMotorMaxSpeed;  // Maximum speed of the wheel
      double maxTurnSpeed;
      double maxTurnAccelleration;
      double maxSpeed;
      double speedupAccelleration;
      double speedupMargin;
    } TribotConfig;

    TribotConfig conf;

    static TribotConfig getDefaultConfig() {
      TribotConfig conf;
      conf.bodyRadius         = 1.;
      conf.bodyHeight         = 1.;
      conf.bodyMass           = 1.;
      conf.wheelRadius        = .6;
      conf.wheelHeight        = .4;
      conf.wheelMass          = 5.;
      conf.wheelMotorPower    = 500.;
      conf.wheelMotorMaxSpeed = 500.;
      conf.maxTurnSpeed = 500;
      conf.maxTurnAccelleration = 500;
      conf.speedupAccelleration = 500;
      conf.speedupMargin = 500;
      return conf;
    }

    /**
     * Constructor
     */
    Tribot(const OdeHandle& odeHandle,
           const OsgHandle& osgHandle,
           const TribotConfig& config = getDefaultConfig(),
           const std::string& name = "Tribot");


    /**
     *
     */
    virtual ~Tribot();

    /**
     * Place Robot in desired Pose
     * @param pose desired 4x4 pose matrix
     */
    virtual void placeIntern(const osg::Matrix& pose);

    /**
     * Create the Robot in desired pose
     * @param pose desired 4x4 pose matrix
     */
    virtual void create(const osg::Matrix& pose);

    void printInfo() {
      std::cout << "Robot: " << getName() << "\n";
    }

    virtual double getWheelToWorldAngle();
  private:
    virtual Primitive * createWheel(int side, const osg::Matrix& pose);
    virtual void createMotor(std::string name, HingeJoint* joint);
    Primitive* lWheel;
    Primitive* rWheel;
  };
}

#endif
