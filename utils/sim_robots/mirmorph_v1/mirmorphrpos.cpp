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

#include "mirmorphrpos.h"
#include <ode_robots/joint.h>
#include <ode_robots/irsensor.h>
#include <ode_robots/primitive.h>
#include <ode_robots/osgprimitive.h>



using namespace osg;
using namespace std;

namespace lpzrobots {

MirmorphRPos::MirmorphRPos(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                           MirmorphRPosConf conf, const std::string& name)
    : Mirmorph(odeHandle, osgHandle, name, conf.mirconf, conf.force, conf.speed), conf(conf)
{
    length=conf.mirconf.length; // length of body

    wheelsubstance=conf.wheelSubstance;
    //added:relative position sensors
    //std::cout << "rpos sensing check reached, for "<< conf.rpos_sensor_references.size() << "sensors \n";
    if (conf.rpos_sensor_references.size()>0) {
        //std::cout << "rpos sensing active with "<< conf.rpos_sensor_references.size() << " sensors \n";
        // Relative position sensor
        for (std::vector<Primitive*>::iterator it = conf.rpos_sensor_references.begin(); it<conf.rpos_sensor_references.end();it++)
        {
            // Maxlen 1 and exponent 1 gives exact distances as linear sensor
            RelativePositionSensor GoalSensor_tmp(1, 1,Sensor::X|Sensor::Y|Sensor::Z, true);
            GoalSensor_tmp.setReference(*it);

            rpos_goal_sensor.push_back(GoalSensor_tmp);
            //        sensorno += rpos_sens_tmp.getSensorNumber(); // increase sensornumber of robot
            //std::cout << "sensor added, now "<< sensorno << " sensors \n";
            // sensorno+=3;

        }
    }

    //outfilePos.open("position.txt");
}


MirmorphRPos::~MirmorphRPos(){
    destroy();
}

Position MirmorphRPos::getPosition(){

    /* Position pos_robot;

         pos_robot = Mirmorph::getPosition();

         outfilePos<<pos_robot.x<<" "<<pos_robot.y<<" "<<pos.robot.z<<endl;*/

    return Mirmorph::getPosition();

}


int MirmorphRPos::getSensorNumberIntern(){
    if(!irSensorBank.isInitialized()){
        fprintf(stderr, "FourWheeled:: place the robot before calling agent->init()!\n");
        assert(irSensorBank.isInitialized());
    }
    if(!fitnessSensors.isInitialized()){
        fprintf(stderr, "FourWheeled:: place the robot before calling agent->init()!\n");
        assert(fitnessSensors.isInitialized());
    }

    if(conf.twoWheelMode){
        assert(Mirmorph::getSensorNumberIntern() == 4);
        //      std::cout << irSensorBank.size() << " ir Sensors, "<< conf.rpos_sensor_references.size() << " position sensors \n";
        //      std::cout << 2 + irSensorBank.size() + conf.rpos_sensor_references.size() <<" Sensors total\n";
        return 2 + irSensorBank.size() + fitnessSensors.size() + conf.rpos_sensor_references.size()*3 + 3;
    }else{
        //      std::cout << irSensorBank.size() << " ir Sensors, "<< conf.rpos_sensor_references.size() << " position sensors \n";
        //      std::cout << 4 << " Wheel speed sensors \n";
        //      std::cout << 4 + irSensorBank.size() + conf.rpos_sensor_references.size() <<" Sensors total \n";
        return 4 + irSensorBank.size() + fitnessSensors.size() + conf.rpos_sensor_references.size()*3 + 3;
    }
}

int MirmorphRPos::getSensorsIntern(sensor* sensors, int sensornumber){
    int len = 0;
    //if(sensornumber!= sensorno ){std::cout<<"warning: sensor number mismatch \n";}
    if(conf.twoWheelMode){
        sensor nimm4s[4];
        Mirmorph::getSensors(nimm4s,4);
        sensors[len++] = (nimm4s[0]+nimm4s[2])/2;
        sensors[len++] = (nimm4s[1]+nimm4s[3])/2;
    } else {
        len = Mirmorph::getSensorsIntern(sensors,4);
    }

    // ask sensorbank for sensor values (from infrared sensors)
    //  sensor+len is the starting point in the sensors array
    if (conf.irFront || conf.irSide || conf.irBack){
        len += irSensorBank.get(sensors+len, sensornumber-len);
    }

    // //added
    if (conf.relPosSensor ) {
        //   //std::cout << "Relative Sense enabled\n ";
        for (std::vector<RelativePositionSensor>::iterator it = rpos_goal_sensor.begin(); it<rpos_goal_sensor.end();it++){
            //     //std::cout << "New Sensor chosen \n";
            std::list<sensor> rps_val = it->getList();
            for (int i=0; i<it->getSensorNumber(); i++){
                sensors[len]=rps_val.back();  // z is taken as first value,
                //           // since it is x in local coordinates (see above)
                //           // y is taken as 2nd sensorvalue
                //           // x is taken as 3rd sensorvalue (is z in local coordinates)
                rps_val.pop_back();
                len++;
                //           //std::cout << len << "  Relative Sensor found \n";
            }
        }
    }


    if(conf.orientation){
        std::list<sensor> ori_list = orientationSensor->getList();

        double ori1, ori2, ori3;

        ori1 = ori_list.front();
        ori_list.pop_front();
        ori2 = ori_list.front();
        ori_list.pop_front();
        ori3 = ori_list.front();
        sensors[len++] = ori1;
        sensors[len++] = ori2;
        sensors[len++] = ori3;
    }

    if (conf.fitnessSensors){
        len += fitnessSensors.get(sensors+len, sensornumber-len);
    }

    //std::cout << len << " sensors found \n";
    return len;
}

int MirmorphRPos::getMotorNumberIntern(){
    if(conf.twoWheelMode)
        return 2;
    else
        return Mirmorph::getMotorNumberIntern();
}

void MirmorphRPos::setMotorsIntern(const motor* motors, int motornumber){
    if(conf.twoWheelMode){
        motor nimm4m[4];
        nimm4m[0] = motors[0];
        nimm4m[2] = motors[0];
        nimm4m[1] = motors[1];
        nimm4m[3] = motors[1];
        Mirmorph::setMotorsIntern(nimm4m,4);
    }else
        Mirmorph::setMotorsIntern(motors,motornumber);

}


void MirmorphRPos::update(){
    Mirmorph::update();
    if(conf.useBumper)
        bumpertrans->update();
    // update sensorbank with infrared sensors
    irSensorBank.update();
    fitnessSensors.update();
}

void MirmorphRPos::sense(GlobalData& globalData) {
    // reset ir sensors to maximum value
    irSensorBank.sense(globalData);
    fitnessSensors.sense(globalData);
    orientationSensor->sense(globalData);
    for (std::vector<RelativePositionSensor>::iterator it = rpos_goal_sensor.begin(); it<rpos_goal_sensor.end();it++){
        it->sense(globalData);
    }
}

/** creates vehicle at desired position
      @param pos struct Position with desired position
  */
void MirmorphRPos::create(const osg::Matrix& pose){
    Mirmorph::create(pose);

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
                    Matrix::rotate(-i*conf.frontIR_angle, Vec3(1,0,0)) *
                    Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                    Matrix::translate(length/2.0, i*width/10, 0),
                    conf.irRangeFront, RaySensor::drawAll);
        }
    }
    if (conf.irSide){ // add right infrared sensors to sensorbank if required
        IRSensor* sensor = new IRSensor();
        irSensorBank.registerSensor(sensor, objects[0],
                //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                Matrix::rotate(-conf.sideTopIR_angle, Vec3(1,0,0)) *
                Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                Matrix::translate(0, width/2, 0),
                conf.irRangeSide, RaySensor::drawAll);
        sensor = new IRSensor();
        irSensorBank.registerSensor(sensor, objects[0],
                //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                Matrix::rotate(-conf.sideBottomIR_angle, Vec3(1,0,0)) *
                Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                Matrix::translate(0, width/2, -height/6),
                conf.irRangeSide, RaySensor::drawAll);
    }

    if (conf.irBack){ // add rear right and rear left infrared sensor to sensorbank if required
        for(int i=-1; i<2; i+=2){
            IRSensor* sensor = new IRSensor();
            irSensorBank.registerSensor(sensor, objects[0],
                    Matrix::rotate(i*conf.backIR_angle, Vec3(1,0,0)) *
                    Matrix::rotate(i*M_PI, Vec3(0,1,0)) *
                    Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                    Matrix::translate(-length/2.0, -i*width/10, 0),
                    conf.irRangeBack, RaySensor::drawAll);
        }
    }
    if (conf.irSide){ // add left infrared sensors to sensorbank if required
        IRSensor* sensor = new IRSensor();
        irSensorBank.registerSensor(sensor, objects[0],
                //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                Matrix::rotate(conf.sideTopIR_angle, Vec3(1,0,0)) *
                Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                Matrix::translate(0, -width/2, 0),
                conf.irRangeSide, RaySensor::drawAll);
        sensor = new IRSensor();
        irSensorBank.registerSensor(sensor, objects[0],
                //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                Matrix::rotate(conf.sideBottomIR_angle, Vec3(1,0,0)) *
                Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                Matrix::translate(0, -width/2, -height/6),
                conf.irRangeSide, RaySensor::drawAll);
    }
    // // Added: Relative position sensor
    if (conf.relPosSensor) {
        for (std::vector<RelativePositionSensor>::iterator it = rpos_goal_sensor.begin(); it<rpos_goal_sensor.end();it++){
            it->init(objects[0]); // connect sensor to main body
        }
    }

    orientationSensor = new AxisOrientationSensor(AxisOrientationSensor::Axis, Sensor::X | Sensor::Y | Sensor::Z);
    orientationSensor->init(objects[0]);

    fitnessSensors.setInitData(odeHandle, osgHandle, TRANSM(0,0,0));
    fitnessSensors.init(0);

    if(conf.fitnessSensors){
        IRSensor* sensor = new IRSensor();
        fitnessSensors.registerSensor(sensor, objects[0],
                Matrix::rotate(M_PI/4, Vec3(1,0,0)) *
                Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                Matrix::translate(length/2.0, -width/10, 0),
                conf.irRangeFront, RaySensor::drawNothing);
        sensor = new IRSensor();
        fitnessSensors.registerSensor(sensor, objects[0],
                Matrix::rotate(-M_PI/4, Vec3(1,0,0)) *
                Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                Matrix::translate(length/2.0, width/10, 0),
                conf.irRangeFront, RaySensor::drawNothing);
        sensor = new IRSensor();
        fitnessSensors.registerSensor(sensor, objects[0],
                //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                Matrix::rotate(-M_PI/6, Vec3(1,0,0)) *
                Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                Matrix::translate(0, width/2, 0),
                conf.irRangeSide, RaySensor::drawNothing);
        sensor = new IRSensor();
        fitnessSensors.registerSensor(sensor, objects[0],
                //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                Matrix::rotate(-5*M_PI/6, Vec3(1,0,0)) *
                Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                Matrix::translate(0, width/2, -height/6),
                conf.irRangeSide, RaySensor::drawNothing);
        sensor = new IRSensor();
        fitnessSensors.registerSensor(sensor, objects[0],
                Matrix::rotate(-M_PI/4, Vec3(1,0,0)) *
                Matrix::rotate(M_PI, Vec3(0,1,0)) *
                Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                Matrix::translate(-length/2.0, width/10, 0),
                conf.irRangeBack, RaySensor::drawNothing);
        sensor = new IRSensor();
        fitnessSensors.registerSensor(sensor, objects[0],
                Matrix::rotate(M_PI/4, Vec3(1,0,0)) *
                Matrix::rotate(-M_PI, Vec3(0,1,0)) *
                Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                Matrix::translate(-length/2.0, -width/10, 0),
                conf.irRangeBack, RaySensor::drawNothing);
        sensor = new IRSensor();
        fitnessSensors.registerSensor(sensor, objects[0],
                //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                Matrix::rotate(5*M_PI/6, Vec3(1,0,0)) *
                Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                Matrix::translate(0, -width/2, -height/6),
                conf.irRangeSide, RaySensor::drawNothing);
        sensor = new IRSensor();
        fitnessSensors.registerSensor(sensor, objects[0],
                //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                Matrix::rotate(M_PI/6, Vec3(1,0,0)) *
                Matrix::rotate(M_PI/2, Vec3(0,1,0)) *
                Matrix::translate(0, -width/2, 0),
                conf.irRangeSide, RaySensor::drawNothing);
    }
}


// returns the joint with index i
Joint* MirmorphRPos::getJoint(int i){
    if(i>3)i=3;
    if(i<0)i=0;
    return joints[i];
}


/** destroys vehicle and space
   */
void MirmorphRPos::destroy(){
    if (created)
        irSensorBank.clear();
    fitnessSensors.clear();
    //   rpos_goal_sensor.clear();
    Mirmorph::destroy();
}

}
