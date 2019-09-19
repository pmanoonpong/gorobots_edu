#include "robot.h"

namespace lilibot_ns
{
    Robot::Robot(int argc, char** argv)
    {
        ros = new RosClass(argc,argv);
        rob = new Lilibot();
        if(!rob->init(ros->getHandle())){
            perror("lilibot init false\n");
        }
        stick = new StStick();
        if(!stick->init(ros->getHandle())){
            perror("ststick init false\n");
        }

        motorValue.resize(rob->motor_num);
        sensorValue.resize(rob->sensor_num);
        ROS_INFO("robot node start successful!\n");
    }

    Robot::~Robot(){
        delete rob;
        delete ros;
        delete stick;
    }

    bool Robot::run(){
        if(ros::ok()){
            rob->getSensorValue(sensorValue);
            ros->readSensorValue(sensorValue);
            ros->writeMotorValue(motorValue);
            rob->setMotorValue(motorValue);
            stick->guide();
            return true;
        }else{
            return false;
        }

    }

}
