//
// Created by sun tao  on 2019/04/10.
//

#include "rosClass.h"
namespace stcontroller{

RosClass::RosClass(int argc, char **argv) {
    // 1) Create a ROS nodes (The name has a random component)
    std::string nodeName("ROSCPG");
    ros::init(argc,argv,nodeName);
    spinner = new ros::AsyncSpinner(2);
    spinner->start();
    if(!ros::master::check())
        ROS_ERROR("ros::master::check() did not pass!");
    node = new ros::NodeHandle();
    init();
    ROS_INFO("simROS just started!");
}

RosClass::~RosClass() {
    ROS_INFO("stbot ROS just terminated!");
    spinner->stop();
    delete spinner;
    ros::shutdown();
	delete rate;
	delete node;
}
void RosClass::rosSleep(){
    rate->sleep();
    }
bool RosClass::init(){
    //2) get parameters
    std::vector<std::string> subscribeTopic;
    if(!node->getParam("subscribeTopic",subscribeTopic)){
        ROS_ERROR("subscribe Topic given (namespace: %s)", node->getNamespace().c_str());
        exit(-1);
    }
    std::vector<std::string> advertiseTopic;
    if(!node->getParam("/advertiseTopic",advertiseTopic)){
        ROS_ERROR("advertise Topic given (namespace: %s)", node->getNamespace().c_str());
        exit(-1);
    }
    int RosRate;
    if(!node->getParam("RosRate",RosRate)){
        ROS_ERROR("RosRate given (namespace: %s)", node->getNamespace().c_str());
        return false;
    }
    if(!node->getParam("controlParam",controlParam)){
        ROS_ERROR("control given (namespace: %s)", node->getNamespace().c_str());
        return false;
    }
    param_num=controlParam.size();
    int leg_num_;
    if(!node->getParam("leg_num",leg_num_)){
        ROS_ERROR("leg_num given (namespace: %s)", node->getNamespace().c_str());
        return false;
    }
    leg_num=(uint8_t)leg_num_;
    int motor_num_;
    if(!node->getParam("motor_num",motor_num_)){
        ROS_ERROR("motor_num given (namespace: %s)", node->getNamespace().c_str());
        return false;
    }
    motor_num=(uint8_t)motor_num_;
    int sensor_num_;
    if(!node->getParam("sensor_num",sensor_num_)){
        ROS_ERROR("sensor_num given (namespace: %s)", node->getNamespace().c_str());
        return false;
    }
    sensor_num=(uint8_t)sensor_num_;
    //4) Subscribe to topics and specify callback functions
    sensorValueSub= node->subscribe(subscribeTopic.at(0), 2, &RosClass::sensorValueCallback, this);
    terminateNodeSub= node->subscribe(subscribeTopic.at(1), 2, &RosClass::terminateNodeCallback, this);

    //5) Initialize publishers
    if(advertiseTopic.size()>0)
        motorValuePub=node->advertise<std_msgs::Float32MultiArray>(advertiseTopic.at(0),2);
    if(advertiseTopic.size()>1)
        neuroNetworkValuePub=node->advertise<std_msgs::Float32MultiArray>(advertiseTopic.at(1),2);


    //9)  init variable
    sensorValue.resize(sensor_num);
    motorValue.resize(motor_num);
    controlParameters.resize(controlParam.size());
    rate = new ros::Rate(RosRate);
    return true;
}

//**----------new verion------------------//
void RosClass::sensorValueCallback(const std_msgs::Float32MultiArray& sensorValue)
{
    for(uint8_t idx=0;idx<sensorValue.data.size();idx++)
        this->sensorValue[idx] = sensorValue.data[idx];
}

void RosClass::terminateNodeCallback(const std_msgs::Bool& termNode)
{
    terminateNode=termNode.data;
    if(terminateNode)
        terminate=true;
}

void RosClass::getSensorValue(std::vector<float>& value)const{
    assert(value.size()==sensor_num);
    for(uint8_t idx=0;idx<sensor_num;idx++)
        value[idx] = sensorValue[idx];
}

void RosClass::setMotorValue(const std::vector<float>& value) {
    assert(value.size()==motor_num);
    std_msgs::Float32MultiArray array;
    array.data.resize(motor_num);
    for (uint8_t idx=0;idx<motor_num;idx++) {
        array.data[idx]=value[idx];
    }
    motorValuePub.publish(array);
}

void RosClass::plotNeuroNetwork(std::vector<float>& data) {
    // publish the motor positions:
    std_msgs::Float32MultiArray array;
    array.data.clear();

    for (unsigned int i = 0; i <= data.size(); ++i)
        array.data.push_back(data[i]);
    neuroNetworkValuePub.publish(array);
}

void RosClass::getParameters(std::vector<float>& param)const{
    assert(param.size()==param_num);
    for(uint8_t idx=0;idx<param_num;idx++)
        node->getParam(controlParam[idx],param.at(idx));
}

void RosClass::setParameters(const std::vector<float>& param){
    assert(param.size()==param_num);
    for(uint8_t idx=0;idx<param_num;idx++)
        node->setParam(controlParam[idx],param.at(idx));
}



//------------old version ------------**//
/*
void RosClass::simulationTimeCallback(const rosgraph_msgs::Clock& simTime)
{
    simulationTime.clock=simTime.clock;
}

void RosClass::terminateNodeCallback(const std_msgs::Bool& termNode)
{
    terminateSimulation=termNode.data;
}

void RosClass::simulationStepDoneCallback(const std_msgs::Bool& _simStepDone)
{
    simStepDone=_simStepDone.data;
}

void RosClass::simulationStateCallback(const std_msgs::Int32& _state)
{
    simState=_state.data;
}
ros::NodeHandle* RosClass::getHandle(){
    return node;
    }
*/

}

