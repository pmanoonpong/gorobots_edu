/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/
#ifndef __FOUR_WHEELED_GRIPPER__
#define __FOUR_WHEELED_GRIPPER__
#include <ode_robots/oderobot.h>
#include <ode_robots/nimm4.h>
#include <ode_robots/raysensorbank.h>

//DSW
#include <ode_robots/gripper.h>

#include <iostream>
#include <fstream>
#include <string.h>

using namespace std;
namespace lpzrobots {

  class Primitive;
  class Hinge2Joint;
  class Joint;

  typedef struct {
    double size;
    double force;
    double speed;
    bool sphereWheels;
    bool useBumper;
    bool twoWheelMode; /// < if true then the robot emulates 2 wheels
    bool irFront;
    bool irBack;
    bool irSide;
    double irRangeFront;
    double irRangeBack;
    double irRangeSide;
    Substance wheelSubstance;
    //added
    bool relPosSensor;
    std::vector<Primitive*> rpos_sensor_references;
    // DSW
    bool useGripper;
  } FourWheeledConfGripper;

  /** Robot is based on nimm4 with
      4 wheels and a capsule like body
  */
  class FourWheeledRPosGripper : public Nimm4{
  public:

    /**
     * constructor of nimm4 robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param conf configuration structure
     * @param name name of the robot
     */
    FourWheeledRPosGripper(const OdeHandle& odeHandle, const OsgHandle& osgHandle, FourWheeledConfGripper conf, const std::string& name = "default");


    static FourWheeledConfGripper getDefaultConf(){
      FourWheeledConfGripper conf;
      conf.size         = 1;
      conf.force        = 3;
      conf.speed        = 15;
      conf.sphereWheels = true;
      conf.twoWheelMode = false;
      conf.useBumper    = false;
      conf.irFront      = true;
      conf.irBack       = false;
      conf.irSide       = false;
      conf.irRangeFront = 3;
      conf.irRangeSide  = 2;
      conf.irRangeBack  = 2;
      conf.wheelSubstance.toRubber(40);
      //added:
      conf.relPosSensor = true;
      conf.rpos_sensor_references.clear();
      // DSW
      conf.useGripper = true;
      return conf;
    }

    virtual ~FourWheeledRPosGripper();

    /**
     * updates the OSG nodes of the vehicle
     */
    virtual void update() override;
    virtual Position getPosition();
    virtual int getSensorNumberIntern() override;
    virtual int getMotorNumberIntern() override;

    virtual int getSensorsIntern(sensor* sensors, int sensornumber) override;

    virtual void setMotorsIntern(const motor* motors, int motornumber)  override;

    virtual void sense(GlobalData& globalData) override;

    // returns the joint with index i
    virtual Joint* getJoint(int i);

		// returns pointer to gripper
		virtual void addGrippables(Primitives objects);
		virtual void removeGrippables(Primitives objects);
		virtual void removeAllGrippables();
    
  protected:
    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();
    //added
    FourWheeledConfGripper conf;
    RaySensorBank irSensorBank; // a collection of ir sensors
    Primitive* bumpertrans;
    Primitive* bumper;
    
    // DSW
    Primitive* gripperArm1;
    Primitive* gripperArmTrans1;
    Primitive* gripperArm2;
    Primitive* gripperArmTrans2;
    Primitive* gripperArm3;
    Primitive* gripperArmTrans3;
    Primitive* gripperArm4;
    Primitive* gripperArmTrans4;
    Primitive* gripperArm5;
    Primitive* gripperArmTrans5;
    Primitive* gripperArmGrip;
    Primitive* gripperArmTransGrip;
    
		Gripper* gripper;
  };

}

#endif
