//Created by Tao at 2019-04-12
//
#ifndef ROSCLASS_H_
#define ROSCLASS_H

#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <control_msgs/JointJog.h>
#include <rosgraph_msgs/Clock.h>
namespace stcontroller{

class RosClass {
private:
	// Node
    ros::NodeHandle * node;
    // Topics

    // Subscribers
    ros::Subscriber sensorValueSub;
    ros::Subscriber terminateNodeSub;
    // Publishers
    ros::Publisher neuroNetworkValuePub;
    ros::Publisher motorValuePub;

    ros::Publisher stopSimPub;
    ros::Publisher enableSyncModePub;
    ros::Publisher triggerNextStepPub;

    // Private Global Variables
    struct timeval tv;
    __time_t currentTime_updatedByTopicSubscriber=0;
    bool simStepDone=true;
    bool terminateNode=false;
    int simState=0;
    std::string RosRateParameter;
    int RosRate;
    ros::Rate* rate;
    ros::AsyncSpinner* spinner;

    // Private Methods
    void terminateNodeCallback(const std_msgs::Bool& termNode);
    void sensorValueCallback(const std_msgs::Float32MultiArray& sensorValue);
    /*
    void simulationStepDoneCallback(const std_msgs::Bool& simStepDoneAns);
    void simulationTimeCallback(const rosgraph_msgs::Clock& simTime);
    void simulationStateCallback(const std_msgs::Int32& stateAns);
    */
public:
    // Public Methods
    RosClass(int argc, char * * argv);
    ~RosClass();
    bool init();

    void setRosParam(const std::string name, float value);
    void getRosParam(const std::string name, float& value);
    void rosSleep();
    void synchronousSimulation(std::string option);

    // Public Global Variables
    std::vector<float> controlParameters;
    std::vector<float> jointPosition;
    std::vector<float> jointVelocity;
    std::vector<float> jointEffort;
    std::vector<float> grf;
    std::vector<float> ori;
    std::vector<float> guide;
	
    rosgraph_msgs::Clock simulationTime;

    std::vector<std::string> controlParam;
    std::vector<float> sensorValue;
    std::vector<float> motorValue;
public:
	void getSensorValue(std::vector<float>& value)const;
	void setMotorValue(const std::vector<float>& value);
    void plotNeuroNetwork(std::vector<float>& value);
    ros::NodeHandle* getHandle();

	void setParameters(const std::vector<float>& param);
	void getParameters(std::vector<float>& param)const;
    uint8_t sensor_num;
	uint8_t leg_num;
	uint8_t motor_num;
    uint8_t param_num;
    bool terminate=false;
};

}
#endif //ROSCLASS_H_

// TODO ros_ok ?? Where to place?
