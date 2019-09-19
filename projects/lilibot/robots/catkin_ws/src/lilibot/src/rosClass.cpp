/*
Author: Sun Tao
Date:2019-4-8
Description:lilibot position controller plugin
 */
#include "rosClass.h"
namespace lilibot_ns{

    RosClass::RosClass(int argc, char** argv){

        std::string nodeName("Lilibot");
        ros::init(argc,argv,nodeName);
        spinner = new ros::AsyncSpinner(2);
        spinner->start();
        if(!ros::master::check())
            ROS_ERROR("ros::master::check() did not pass!");

        node = new ros::NodeHandle();
        ros::Duration period(1.0/333); // 333Hz update rate
        init();
        ROS_INFO("Lilibot Robot RosClass");
    }
    RosClass::~RosClass(){
        cpgValueSub.shutdown();
        reflexValueSub.shutdown();
        spinner->stop();
        delete spinner;
        delete node;
        ros::shutdown();
    }

    //puiblic----------------------------
    bool RosClass::init()
    {
        // get joint name from the parameter server
        std::vector<std::string> joint_name;
        if (!node->getParam("joints", joint_name)){
            ROS_ERROR("No joint given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        //imu
        std::string imu_name;
        if (!node->getParam("imu", imu_name)){
            ROS_ERROR("No imu given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        //foot
        std::vector<std::string> foot_name;
        if (!node->getParam("legs", foot_name)){
            ROS_ERROR("No foot given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        //num_motor
        if (!node->getParam("motor_num", motor_num)){
            ROS_ERROR("motr_num given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        //num_sensor
        if (!node->getParam("sensor_num", sensor_num)){
            ROS_ERROR("motr_num given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        //num_motor
        int leg_num;
        if (!node->getParam("leg_num", leg_num)){
            ROS_ERROR("motr_num given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }


        // subscribe topic
        std::vector<std::string> subscribe_name;
        if (!node->getParam("subscribe", subscribe_name)){
            ROS_ERROR("No subscribe given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        // advertise topic
        std::vector<std::string> advertise_name;
        if (!node->getParam("advertise", advertise_name)){
            ROS_ERROR("No advertise given (namespace: %s)", node->getNamespace().c_str());
            return false;
        }
        //pub
        sensorValuePub=node->advertise<std_msgs::Float32MultiArray>(advertise_name.at(0), 1);
        // Start cpgCommand subscriber
        cpgValueSub= node->subscribe<std_msgs::Float32MultiArray>(subscribe_name.at(0), 1, &RosClass::motorValueCallback, this);

        // Start cpgCommand subscriber
        reflexValueSub= node->subscribe<std_msgs::Float32MultiArray>(subscribe_name.at(1), 1, &RosClass::reflexValueCallback, this);
        //init variable
        motorValue.resize(motor_num);
        cpgValue.resize(motor_num);
        reflexValue.resize(motor_num);
        sensorValue.resize(sensor_num);

        ROS_INFO("LilibotController init successful!");
        return true;
    }

    void RosClass::readSensorValue(const std::vector<sensor>& data)
    {   assert(data.size()==sensor_num);
        std_msgs::Float32MultiArray values;
        values.data.resize(sensor_num);
        for(uint8_t idx=0;idx<sensor_num;idx++)
            values.data[idx]=data[idx];
        sensorValuePub.publish(values);
    }

    void RosClass::writeMotorValue(std::vector<command>& data){
        assert(data.size()==motor_num);
        updateMotorValue();
        for(uint8_t idx=0;idx<motor_num;idx++)
            data[idx]=motorValue[idx];
    }
    void RosClass::motorValueCallback(const std_msgs::Float32MultiArray array){
        assert(array.data.size()==motor_num);
        for(uint8_t idx=0;idx<motor_num;idx++)
            cpgValue[idx]=array.data[idx];

    }
    void RosClass::reflexValueCallback(const std_msgs::Float32MultiArray array){
        assert(array.data.size()==motor_num);
        for(uint8_t idx=0;idx<motor_num;idx++)
            reflexValue[idx]=array.data[idx];

    }

    void RosClass::updateMotorValue(){
        for(uint8_t idx=0;idx<motor_num;idx++)
            motorValue[idx]=cpgValue[idx];//reflexValue[idx];// + cpgValue[idx];
            //motorValue[idx]=reflexValue[idx];// + cpgValue[idx];
    }
    ros::NodeHandle* RosClass::getHandle(){
        return node;
    }
}//namespace
