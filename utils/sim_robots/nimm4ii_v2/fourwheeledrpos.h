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
#include <ode_robots/nimm4.h>
#include <ode_robots/raysensorbank.h>
//added

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
  } FourWheeledConf;

  /** Robot is based on nimm4 with
      4 wheels and a capsule like body
  */
  class FourWheeledRPos : public Nimm4{
  public:

    /**
     * constructor of nimm4 robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param conf configuration structure
     * @param name name of the robot
     */
    FourWheeledRPos(const OdeHandle& odeHandle, const OsgHandle& osgHandle, FourWheeledConf conf, const std::string& name = "default");


    static FourWheeledConf getDefaultConf(){
      FourWheeledConf conf;
      conf.size         = 1;
      conf.force        = 3;
      conf.speed        = 15;
      conf.sphereWheels = true;
      conf.twoWheelMode = false;
      conf.useBumper    = true;
      conf.irFront      = true;
      conf.irBack       = true;
      conf.irSide       = true;
      conf.irRangeFront = 3;
      conf.irRangeSide  = 2;
      conf.irRangeBack  = 2;
      conf.wheelSubstance.toRubber(40);
      //added:
      conf.relPosSensor = true;
      conf.rpos_sensor_references.clear();
      return conf;
    }

    virtual ~FourWheeledRPos();

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

  protected:
    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();
    //added
    //    std::vector<RelativePositionSensor> rpos_sensor;
    FourWheeledConf conf;
    RaySensorBank irSensorBank; // a collection of ir sensors
    Primitive* bumpertrans;
    Primitive* bumper;
  };

}

#endif
