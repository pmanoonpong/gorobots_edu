#ifndef _LILIBOT_H_
#define _LILIBOT_H_
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_controllers/position_control.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <sstream>
#include <algorithm>
#include "typeHeader.h"
using namespace std;
namespace lilibot_ns{

    class Lilibot{
        public:
            Lilibot();
            ~Lilibot();    
            bool init(ros::NodeHandle* node);

            void setMotorValue(const vector<command>& cmd_pos,const vector<command>& cmd_vel, const vector<command>& cmd_eff);
            void setMotorValue(const vector<command>& value);
            void getSensorValue(vector<sensor>& value);
         private:
            void initMsg(); 
            void readJoints();
            void readImu();
            void readFootForce();
            void writeMotorValue(vector<int32_t>& pos, vector<int32_t>& eff);
            void localController();
            void value2cmd(const std::vector<float>& value, std::vector<int32_t>& cmd);
            void getParameters();
        private:
            DynamixelWorkbench * dxl_wb_;
            serial::Serial *ser; //声明串口对象 
            ros::NodeHandle* node;
            long int t;
            //deivce num
            //feedback sensory
            std::vector<sensor> position;
            std::vector<sensor> velocity;
            std::vector<sensor> current;

            std::vector<sensor> ori;
            std::vector<sensor> grf;
            std::vector<sensor> sensorValue;
            std::vector<sensor> motorValue;

            //dxl device
            std::vector<int32_t> goal_pos;
            std::vector<int32_t> goal_vel;
            std::vector<int32_t> goal_cur;

            std::vector<int32_t> pre_pos;
            std::vector<int32_t> pre_vel;
            std::vector<int32_t> pre_cur;

            //local controller
            std::vector<int32_t> pos_err;
            std::vector<int32_t> pre_pos_err;
            std::vector<float> calc_tor;
            std::vector<int32_t> goal_tor;

            float p_gain;
            float d_gain;
            float MI;

            uint8_t dxl_id_[16];
            uint8_t dxl_cnt_;
        public:
            int leg_num;
            int motor_num;
            int sensor_num;
            int ori_num;

    };

}

#endif

