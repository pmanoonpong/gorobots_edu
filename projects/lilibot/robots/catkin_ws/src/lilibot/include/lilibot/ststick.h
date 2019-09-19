#ifndef _STSTICK_H_
#define _STSTICK_H_
#include <stdio.h>
#include "std_msgs/String.h"
#include <stdlib.h>
#include "joystick.h"
#include <sstream>
#include <algorithm>
#include <ros/ros.h>

using namespace std;
typedef struct{
    bool event;
    bool isButton;
    bool isAxis;
    int32_t button;
    int32_t axis;
    int32_t valueOffset;
}StickCommand;


class StStick{    
    private:
        Joystick *joystick;   
        StickCommand GCommand;
        void stickEvent();
        ros::NodeHandle* node_handle;
    public:
        StStick();
        ~StStick();
        bool init(ros::NodeHandle* node);
        void guide();
};

#endif
