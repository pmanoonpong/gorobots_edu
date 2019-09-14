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
#include <ode-dbl/ode.h>
#include <assert.h>
#include <osg/Matrix>

#include "fourwheeledrpos_gripper.h"
#include <ode_robots/joint.h>
#include <ode_robots/irsensor.h>
#include <ode_robots/primitive.h>
#include <ode_robots/osgprimitive.h>

#include "relativepositionsensor.h"

// DSW
#include <ode_robots/gripper.h>

using namespace osg;
using namespace std;

namespace lpzrobots {

  FourWheeledRPosGripper::FourWheeledRPosGripper(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
			   FourWheeledConfGripper conf, const std::string& name)
    : Nimm4(odeHandle, osgHandle, name, conf.size, conf.force, conf.speed, conf.sphereWheels), conf(conf)
  {
    length=conf.size/2.0; // length of body
    wheelsubstance=conf.wheelSubstance;

    //added:relative position sensors
    if (conf.rpos_sensor_references.size()>0) {
      // Relative position sensor
      FOREACHIa(conf.rpos_sensor_references, ref, i){
        auto rpos_sens_tmp = make_shared<RelativePositionSensor>(
            1 /*max distance for normalization*/ ,
            1 /*exponent for sensor characteristic*/,
            /*dimensions to sense*/
            Sensor::XYZ,
            // actually it should be only YZ since X points upwards, but the controller does not fit.
            // use Z as x-coordinate (robot's capsule has z axis along the axis and this local Z points forwards)
            /*local_coordinates*/ true);
        rpos_sens_tmp->setReference(*ref);
        rpos_sens_tmp->setBaseName("Pos Target " + itos(i) + "-");
        addSensor(rpos_sens_tmp);
      }
      }
  };


  FourWheeledRPosGripper::~FourWheeledRPosGripper(){
    destroy();
  }

  Position FourWheeledRPosGripper::getPosition(){
	  return Nimm4::getPosition();
  }


  int FourWheeledRPosGripper::getSensorNumberIntern(){
    if(!irSensorBank.isInitialized()){
      fprintf(stderr, "FourWheeled:: place the robot before calling agent->init()!\n");
      assert(irSensorBank.isInitialized());
    }

    if(conf.twoWheelMode){
      assert(Nimm4::getSensorNumberIntern() == 4);
      return 2 + irSensorBank.size();
    }else{
      return 4 + irSensorBank.size();
    }
 }

  int FourWheeledRPosGripper::getSensorsIntern(sensor* sensors, int sensornumber){
    int len = 0;
    if(conf.twoWheelMode){
      sensor nimm4s[4];
      Nimm4::getSensors(nimm4s,4);
      sensors[len++] = (nimm4s[0]+nimm4s[2])/2;
      sensors[len++] = (nimm4s[1]+nimm4s[3])/2;
    } else {
      len = Nimm4::getSensorsIntern(sensors,4);
    }
    // ask sensorbank for sensor values (from infrared sensors)
    //  sensor+len is the starting point in the sensors array
    if (conf.irFront || conf.irSide || conf.irBack){
      len += irSensorBank.get(sensors+len, sensornumber-len);
    }

    return len;
  };

  int FourWheeledRPosGripper::getMotorNumberIntern(){
    if(conf.twoWheelMode)
      return 2;
    else
      return Nimm4::getMotorNumberIntern();
  }

  void FourWheeledRPosGripper::setMotorsIntern(const motor* motors, int motornumber){
    if(conf.twoWheelMode){
      motor nimm4m[4];
      nimm4m[0] = motors[0];
      nimm4m[2] = motors[0];
      nimm4m[1] = motors[1];
      nimm4m[3] = motors[1];
      Nimm4::setMotorsIntern(nimm4m,4);
    }else
      Nimm4::setMotorsIntern(motors,motornumber);

  }


  void FourWheeledRPosGripper::update(){
    Nimm4::update();
    if(conf.useBumper)
      bumpertrans->update();
    // DSW update gripper 
    if(conf.useGripper) {
    	gripperArmTrans1->update();
			gripperArmTrans2->update();
			gripperArmTrans3->update();
			gripperArmTrans4->update();
			gripperArmTrans5->update();
			gripperArmTransGrip->update();
    }
    // update sensorbank with infrared sensors
    irSensorBank.update();
  }

  void FourWheeledRPosGripper::sense(GlobalData& globalData) {
    // reset ir sensors to maximum value
    irSensorBank.sense(globalData);
  }

  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void FourWheeledRPosGripper::create(const osg::Matrix& pose){
    Nimm4::create(pose);
    // create frame to not fall on back

    if(conf.useBumper){
      bumper = new Box(0.1 , width+2*wheelthickness+radius, length+0.7*width);
      bumper->setTexture("Images/wood.rgb");
      bumpertrans = new Transform(objects[0], bumper,
                                  Matrix::translate(width*0.6-radius, 0, 0));
      bumpertrans->init(odeHandle, 0, osgHandle);
    }

		// DSW Build gripper
    if(conf.useGripper){

			gripperArm1 = new Box(0.1 , 0.3, 0.5);
			gripperArm1->setTexture("Images/wood.rgb");		
			gripperArmTrans1 = new Transform(objects[0],gripperArm1, Matrix::translate(0,0,0.25) * Matrix::rotate(M_PI/4,Vec3(0,1,0)));
			gripperArmTrans1->init(odeHandle, 0, osgHandle.changeColor(Color(2, 156/255.0, 0, 1.0f)));
			
			gripperArm2 = new Box(0.1 , 0.695, 0.5);
			gripperArm2->setTexture("Images/wood.rgb");		
			gripperArm2->setSubstance(Substance(5.0,10.0,99.0,1.0));
			gripperArmTrans2 = new Transform(objects[0],gripperArm2, Matrix::translate(0.35,-0.35,0.55));
			gripperArmTrans2->init(odeHandle, 0, osgHandle.changeColor(Color(2, 156/255.0, 0, 1.0f)));
			
			gripperArm3 = new Box(0.1 , 0.695, 0.5);
			gripperArm3->setTexture("Images/wood.rgb");
			gripperArm3->setSubstance(Substance(5.0,10.0,99.0,1.0));		
			gripperArmTrans3 = new Transform(objects[0],gripperArm3, Matrix::translate(0.35,0.35,0.55));
			gripperArmTrans3->init(odeHandle, 0, osgHandle.changeColor(Color(2, 156/255.0, 0, 1.0f)));
			
			gripperArm4 = new Box(0.1 , 0.1, 0.6);
			gripperArm4->setTexture("Images/wood.rgb");		
			gripperArmTrans4 = new Transform(objects[0],gripperArm4, Matrix::translate(0.35,0.475,0.9) * Matrix::rotate(-M_PI/10,Vec3(1,0,0)));
			gripperArmTrans4->init(odeHandle, 0, osgHandle.changeColor(Color(2, 156/255.0, 0, 1.0f)));
			
			gripperArm5 = new Box(0.1 , 0.1, 0.6);
			gripperArm5->setTexture("Images/wood.rgb");		
			gripperArmTrans5 = new Transform(objects[0],gripperArm5, Matrix::translate(0.35,-0.475,0.9) * Matrix::rotate(M_PI/10,Vec3(1,0,0)));
			gripperArmTrans5->init(odeHandle, 0, osgHandle.changeColor(Color(2, 156/255.0, 0, 1.0f)));
			
			gripperArmGrip = new Box(0.1 , 0.01, 0.5);
			gripperArmGrip->setTexture("Images/wood.rgb");		
			gripperArmTransGrip = new Transform(objects[0],gripperArmGrip, Matrix::translate(0.35,0,0.55));
			gripperArmTransGrip->init(odeHandle, 0, osgHandle.changeColor(Color(0.8,0.8,0.8)));

			// create gripper material and attach it to primitive
			GripperConf grippConf = Gripper::getDefaultConf();
			grippConf.gripDuration = 999.0;
			grippConf.releaseDuration = 0.0;
			grippConf.forbitLastPrimitive = false;
			gripper = new Gripper(grippConf);
			gripper->attach(gripperArmTransGrip);
		
    }


    /* initialize sensorbank (for use of infrared sensors)
     * sensor values (if sensors used) are saved in the vector of
     * sensorvalues in the following order:
     * front left
     * front right
     * right middle
     * rear right
     * rear left
     * left  middle
    */
    irSensorBank.setInitData(odeHandle, osgHandle, TRANSM(0,0,0));
    irSensorBank.init(0);

	// DSW 4 front laser sensors pointing downwards to detect the gap
	if (conf.irFront){ // add front left and front right infrared sensor to sensorbank if required
/*		for(int i=-1; i<2; i+=2){ //for(int i=-1; i<2; i+=2){ 
			IRSensor* sensor = new IRSensor();
			irSensorBank.registerSensor(sensor, objects[0],
			  //Matrix::rotate(-M_PI/8, Vec3(0,1,0)) * // DSW pointing downwards
			  Matrix::rotate(i*M_PI/10, Vec3(1,0,0)) *
			  Matrix::translate(0,-i*width/10,length/2 + width/2 - width/60 ),
			  conf.irRangeFront, RaySensor::drawAll);
      	}
*/
		// frontal sensor
		IRSensor* sensor = new IRSensor();
		irSensorBank.registerSensor(sensor, objects[0],
		//Matrix::rotate(-M_PI/8, Vec3(0,1,0)) * // DSW pointing downwards
		//Matrix::rotate(i*M_PI/10, Vec3(1,0,0)) *
		  Matrix::translate(0,0,length/2 + width/2 - width/60 ),
		  conf.irRangeFront, RaySensor::drawAll);
		
		// Frontal sensor used as touch sensor on gripper
		IRSensor* sensor1 = new IRSensor();
		irSensorBank.registerSensor(sensor1, objects[0],
		//Matrix::rotate(-M_PI/8, Vec3(0,1,0)) * // DSW pointing downwards
		//Matrix::rotate(i*M_PI/10, Vec3(1,0,0)) *
		  Matrix::translate(0.35,0, 0.799),
		  conf.irRangeFront, RaySensor::drawAll);
		
		
		for(int i=-3; i<4; i+=6){ //for(int i=-1; i<2; i+=2){ // DSW added 2 more sensors
			IRSensor* sensor = new IRSensor();
			irSensorBank.registerSensor(sensor, objects[0],
			  Matrix::rotate(-M_PI/7.7, Vec3(0,1,0)) * // DSW pointing downwards
			  Matrix::rotate(i*M_PI/7.7, Vec3(1,0,0)) *
			  Matrix::translate(0,-i*width/10,length/2 + width/2 - width/60 ),
			  conf.irRangeFront, RaySensor::drawAll);
      	}
		for(int i=-3; i<4; i+=6){ //for(int i=-1; i<2; i+=2){ // DSW added 2 more sensors
			IRSensor* sensor = new IRSensor();
			irSensorBank.registerSensor(sensor, objects[0],
			  Matrix::rotate(-M_PI/7, Vec3(0,1,0)) * // DSW pointing downwards
			  Matrix::rotate(i*M_PI/7, Vec3(1,0,0)) *
			  Matrix::translate(0,-i*width/10,length/2 + width/2 - width/60 ),
			  conf.irRangeFront, RaySensor::drawAll);
      	}
    }
    if (conf.irSide){ // add right infrared sensors to sensorbank if required
		IRSensor* sensor = new IRSensor();
		irSensorBank.registerSensor(sensor, objects[0],
			//Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
			Matrix::rotate(M_PI/4, Vec3(1,0,0)) *
			Matrix::translate(0,-width/2, 0 ),
			conf.irRangeSide, RaySensor::drawAll);
		sensor = new IRSensor();
		irSensorBank.registerSensor(sensor, objects[0],
			//Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
			Matrix::rotate(M_PI/6, Vec3(1,0,0)) *
			Matrix::translate(-width/6,-width/2, 0 ),
			conf.irRangeSide, RaySensor::drawAll);
    }

    if (conf.irBack){ // add rear right and rear left infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
			IRSensor* sensor = new IRSensor();
			irSensorBank.registerSensor(sensor, objects[0],
				Matrix::rotate(-i*M_PI/10, Vec3(1,0,0)) *
				Matrix::rotate(i*M_PI, Vec3(0,1,0)) *
				Matrix::translate(0,i*width/10,-(length/2 + width/2 - width/60) ),
				conf.irRangeBack, RaySensor::drawAll);
      }
    }
	if (conf.irSide){ // add left infrared sensors to sensorbank if required
		IRSensor* sensor = new IRSensor();
		irSensorBank.registerSensor(sensor, objects[0],
			//Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
			Matrix::rotate(-M_PI/4, Vec3(1,0,0)) *
			Matrix::translate(0,width/2, 0),
			conf.irRangeSide, RaySensor::drawAll);
		sensor = new IRSensor();
		irSensorBank.registerSensor(sensor, objects[0],
			//Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
			Matrix::rotate(-M_PI/6, Vec3(1,0,0)) *
			Matrix::translate(-width/6,width/2, 0),
			conf.irRangeSide, RaySensor::drawAll);
    }
  };


  // returns the joint with index i
  Joint* FourWheeledRPosGripper::getJoint(int i){
    if(i>3)i=3;
    if(i<0)i=0;
    return joints[i];
  }

	// DSW
	void FourWheeledRPosGripper::addGrippables(Primitives objects){
		gripper->addGrippables(objects);
	}

	void FourWheeledRPosGripper::removeGrippables(Primitives objects){
		gripper->removeGrippables(objects);
	}
	
	void FourWheeledRPosGripper::removeAllGrippables(){
		gripper->removeAllGrippables();
	}

  /** destroys vehicle and space
   */
  void FourWheeledRPosGripper::destroy(){
    if (created)
      irSensorBank.clear();
    Nimm4::destroy();
  }

}
