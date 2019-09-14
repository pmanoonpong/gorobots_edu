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
#ifndef __FOUR_WHEELED__
#define __FOUR_WHEELED__
#include <ode_robots/oderobot.h>
#include <ode_robots/mirmorph.h>
#include <ode_robots/raysensorbank.h>
#include <selforg/inspectable.h>
#include <ode_robots/contactsensor.h>
#include <ode_robots/axisorientationsensor.h>
//added

#include <iostream>
#include <fstream>
#include <string.h>

#include "relativepositionsensor.h"

#define MAX_OR M_PI/3
#define MIN_OR -M_PI/3

using namespace std;
namespace lpzrobots {

  class Primitive;
  class Hinge2Joint;
  class Joint;

  typedef struct {
    lpzrobots::MirmorphConf mirconf;
    double force;
    double speed;
    bool useBumper;
    bool twoWheelMode; /// < if true then the robot emulates 2 wheels
    bool irFront;
    bool irBack;
    bool irSide;
    bool orientation;
    bool fitnessSensors;
    double irRangeFront;
    double irRangeBack;
    double irRangeSide;
    Substance wheelSubstance;
    //added
    bool relPosSensor;
    std::vector<Primitive*> rpos_sensor_references;
    double frontIR_angle;
    double sideTopIR_angle;
    double sideBottomIR_angle;
    double backIR_angle;
  } MirmorphRPosConf;

  /** Robot is based on nimm4 with
      4 wheels and a capsule like body
  */
  class MirmorphRPos : public Mirmorph{
  public:

    /**
     * constructor of nimm4 robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param conf configuration structure
     * @param name name of the robot
     */
    MirmorphRPos(const OdeHandle& odeHandle, const OsgHandle& osgHandle, MirmorphRPosConf conf, const std::string& name = "default");


    static MirmorphRPosConf getDefaultConf(){
      MirmorphRPosConf conf;
      conf.mirconf.height = 0.35;
      conf.mirconf.length = 1.6;
      conf.mirconf.width = 1.2;
      for(int i=0; i<4; i++){
        conf.mirconf.radius.push_back(0.3);
        conf.mirconf.wheelthickness.push_back(0.1);
        conf.mirconf.wheelheights.push_back(conf.mirconf.height/2.0);

      }
      conf.mirconf.sphereWheels = false;
      conf.force        = 5;
      conf.speed        = 15;
      conf.twoWheelMode = false;
      conf.useBumper    = false;
      conf.irFront      = true;
      conf.irBack       = true;
      conf.irSide       = true;
      conf.orientation  = true;
      conf.fitnessSensors       = true;
      conf.irRangeFront = 4;
      conf.irRangeSide  = 4;
      conf.irRangeBack  = 3;
      conf.wheelSubstance.toRubber(40);
      //added:
      conf.relPosSensor = true;
      conf.rpos_sensor_references.clear();
      conf.frontIR_angle = M_PI/10;
      conf.sideTopIR_angle = M_PI/4;
      conf.sideBottomIR_angle = M_PI/6;
      conf.backIR_angle = M_PI/10;
      return conf;
    }

    virtual ~MirmorphRPos();

    /**
     * updates the OSG nodes of the vehicle
     */
    virtual void update() override;
    virtual Position getPosition();
    virtual int getSensorNumberIntern() override;
    virtual int getMotorNumberIntern() override;

    virtual int getSensorsIntern(sensor* sensors, int sensornumber) override;

    virtual void setMotorsIntern(const motor* motors, int motornumber)  override;

    virtual void sense(GlobalData& globalData) override;   //replaces old doInternalStuff function

    // returns the joint with index i
    virtual Joint* getJoint(int i);

    std::vector<RelativePositionSensor> rpos_sensor;

  protected:
    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();
    //added

    MirmorphRPosConf conf;

    Primitive* bumpertrans;
    Primitive* bumper;

  private:
    RaySensorBank irSensorBank; // a collection of ir sensors
    RaySensorBank fitnessSensors;
    std::vector<RelativePositionSensor> rpos_goal_sensor;
    AxisOrientationSensor* orientationSensor;

  };

}

#endif
