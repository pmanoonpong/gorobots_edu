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

#include "fourwheeledrpos.h"
#include <ode_robots/joint.h>
#include <ode_robots/irsensor.h>
#include <ode_robots/primitive.h>
#include <ode_robots/osgprimitive.h>

#include "relativepositionsensor.h"

using namespace osg;
using namespace std;

namespace lpzrobots {

  FourWheeledRPos::FourWheeledRPos(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
			   FourWheeledConf conf, const std::string& name)
    : Nimm4(odeHandle, osgHandle, name, conf.size, conf.force, conf.speed, conf.sphereWheels), conf(conf)
  {
    length=conf.size/2.0; // length of body

    wheelsubstance=conf.wheelSubstance;
    //added:relative position sensors
    //std::cout << "rpos sensing check reached, for "<< conf.rpos_sensor_references.size() << "sensors \n";
    if (conf.rpos_sensor_references.size()>0) {
      //std::cout << "rpos sensing active with "<< conf.rpos_sensor_references.size() << " sensors \n";
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
        //        rpos_sensor.push_back(rpos_sens_tmp);
        //        sensorno += rpos_sens_tmp.getSensorNumber(); // increase sensornumber of robot
        //std::cout << "sensor added, now "<< sensorno << " sensors \n";
       // sensorno+=3;

      }
      }

   //outfilePos.open("position.txt");
  };


  FourWheeledRPos::~FourWheeledRPos(){
    destroy();
  }

  Position FourWheeledRPos::getPosition(){

	/* Position pos_robot;

	 pos_robot = Nimm4::getPosition();

	 outfilePos<<pos_robot.x<<" "<<pos_robot.y<<" "<<pos.robot.z<<endl;*/

	  return Nimm4::getPosition();

  }


  int FourWheeledRPos::getSensorNumberIntern(){
    if(!irSensorBank.isInitialized()){
      fprintf(stderr, "FourWheeled:: place the robot before calling agent->init()!\n");
      assert(irSensorBank.isInitialized());
    }

    if(conf.twoWheelMode){
      assert(Nimm4::getSensorNumberIntern() == 4);
//      std::cout << irSensorBank.size() << " ir Sensors, "<< conf.rpos_sensor_references.size() << " position sensors \n";
//      std::cout << 2 + irSensorBank.size() + conf.rpos_sensor_references.size() <<" Sensors total\n";
      return 2 + irSensorBank.size();
    }else{
//      std::cout << irSensorBank.size() << " ir Sensors, "<< conf.rpos_sensor_references.size() << " position sensors \n";
//      std::cout << 4 << " Wheel speed sensors \n";
//      std::cout << 4 + irSensorBank.size() + conf.rpos_sensor_references.size() <<" Sensors total \n";
      return 4 + irSensorBank.size();
    }
 }

  int FourWheeledRPos::getSensorsIntern(sensor* sensors, int sensornumber){
    int len = 0;
   //if(sensornumber!= sensorno ){std::cout<<"warning: sensor number mismatch \n";}
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

    // //added
    // if (conf.relPosSensor) {
    //   //std::cout << "Relative Sense enabled\n ";
    //   for (std::vector<RelativePositionSensor>::iterator it = rpos_sensor.begin(); it<rpos_sensor.end();it++){
    //     //std::cout << "New Sensor chosen \n";
    //     std::list<sensor> rps_val = it->getList();
    //         for (int i=0; i<it->getSensorNumber(); i++){
    //           sensors[len]=rps_val.back();  // z is taken as first value,
    //           // since it is x in local coordinates (see above)
    //           // y is taken as 2nd sensorvalue
    //           // x is taken as 3rd sensorvalue (is z in local coordinates)
    //           rps_val.pop_back();
    //           len++;
    //           //std::cout << len << "  Relative Sensor found \n";
    //         }
    //   }
    // }

    //std::cout << len << " sensors found \n";
    return len;
  };

  int FourWheeledRPos::getMotorNumberIntern(){
    if(conf.twoWheelMode)
      return 2;
    else
      return Nimm4::getMotorNumberIntern();
  }

  void FourWheeledRPos::setMotorsIntern(const motor* motors, int motornumber){
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


  void FourWheeledRPos::update(){
    Nimm4::update();
    if(conf.useBumper)
      bumpertrans->update();
    // update sensorbank with infrared sensors
    irSensorBank.update();
  }

  void FourWheeledRPos::sense(GlobalData& globalData) {
    // reset ir sensors to maximum value
    irSensorBank.sense(globalData);
  }

  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void FourWheeledRPos::create(const osg::Matrix& pose){
    Nimm4::create(pose);
    // create frame to not fall on back

    if(conf.useBumper){
      bumper = new Box(0.1 , width+2*wheelthickness+radius, length+0.7*width);
      bumper->setTexture("Images/wood.rgb");
      bumpertrans = new Transform(objects[0], bumper,
                                  Matrix::translate(width*0.6-radius, 0, 0));
      bumpertrans->init(odeHandle, 0, osgHandle);
      //delete bumpertrans;
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

    if (conf.irFront){ // add front left and front right infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
	IRSensor* sensor = new IRSensor();
	irSensorBank.registerSensor(sensor, objects[0],
				    Matrix::rotate(i*M_PI/10, Vec3(1,0,0)) *
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
    // // Added: Relative position sensor
    // if (conf.relPosSensor) {
    //   for (std::vector<RelativePositionSensor>::iterator it = rpos_sensor.begin(); it<rpos_sensor.end();it++){
    //     it->init(objects[0]); // connect sensor to main body
    //   }
    // }
  };


  // returns the joint with index i
  Joint* FourWheeledRPos::getJoint(int i){
    if(i>3)i=3;
    if(i<0)i=0;
    return joints[i];
  }


  /** destroys vehicle and space
   */
  void FourWheeledRPos::destroy(){
    if (created)
      irSensorBank.clear();
    Nimm4::destroy();
  }

}
