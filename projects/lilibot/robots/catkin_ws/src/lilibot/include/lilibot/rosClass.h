#ifndef ROSCLASSLILIBOT_H_
#define ROSCLASSLILIBOT_H_

#include <cstdio>
#include <cstdlib>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <control_msgs/JointJog.h>
#include <ros/spinner.h>
#include "typeHeader.h"
namespace lilibot_ns{
    class RosClass{
        public:
            RosClass(int argc,char** argv);
            ~RosClass();
            bool init();

        private:
            ros::Publisher sensorValuePub;
            ros::Subscriber cpgValueSub;
            ros::Subscriber reflexValueSub;

            void motorValueCallback(const std_msgs::Float32MultiArray array);
            void reflexValueCallback(const std_msgs::Float32MultiArray array);
            void updateMotorValue();
            ros::AsyncSpinner* spinner;
            ros::NodeHandle* node;
            
            std::vector<command> motorValue;
            std::vector<command> reflexValue;
            std::vector<command> cpgValue;
            std::vector<command> sensorValue;
        public:

            void readSensorValue(const std::vector<sensor>& data);
            void writeMotorValue(std::vector<command>& data);
            ros::NodeHandle* getHandle();
            int motor_num;
            int sensor_num;
    };
}//namespace

#endif
