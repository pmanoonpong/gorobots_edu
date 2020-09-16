//*******************************************
//*                                         *
//*        Adaptive Locomotion Control      *
//*                                         *
//*******************************************
// update: 03/09/2020
// version: 1.0.0


// standard ros library
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <rosgraph_msgs/Clock.h>
#include <cmath>
#include <iostream>
#include <array>
#include <termios.h>

using namespace std;

//*******************************************
//*                                         *
//*            define parameter             *
//*                                         *
//*******************************************

#define RATE 10 // reflesh rate of ros


//*******************************************
//*                                         *
//*               ros variable              *
//*                                         *
//*******************************************

// buffer (array of float) to store signal before publish to ros
std_msgs::Float32MultiArray keySignal;


//*******************************************
//*                                         *
//*            global variable              *
//*                                         *
//*******************************************

float Key_out1 = 0;
float Key_out2 = 0;
float Key_out3 = 0;
float Key_out4 = 0;

//*******************************************
//*                                         *
//*            global function              *
//*                                         *
//*******************************************

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


//*******************************************
//*                                         *
//*              main program               *
//*                                         *
//*******************************************

int main(int argc, char *argv[]){

    // create ros node
    std::string nodeName("keyboard");
    ros::init(argc,argv,nodeName);

    // check robot operating system
    if(!ros::master::check())
        ROS_ERROR("ros::master::check() did not pass!");
    ros::NodeHandle node("~");

    ROS_INFO("simROS just started!");

    // set reflesh rate
    ros::Rate* rate;
    rate = new ros::Rate(RATE);
    ros::Rate loop_rate(RATE);

    //*******************************************
    //*                                         *
    //*    define publisher and subscriber      *
    //*                                         *
    //*******************************************

    ros::Publisher outputKey;
    outputKey = node.advertise<std_msgs::Float32MultiArray>("/keyboard_topic",1);

    while(ros::ok())
    {

        //*******************************************
        //*                                         *
        //*              ros parameter              *
        //*                                         *
        //*******************************************

        float Key_out1 = 0;
        float Key_out2 = 0;        
        float Key_out3 = 0;
        float Key_out4 = 0;

        int c = getch();   // call your non-blocking input function
        if (c == 'a')
        {
            ROS_INFO("left");
            Key_out1 = 1;
        }
        else if (c == 'd')
        {
            ROS_INFO("right");
            Key_out2 = 1;
        }
        else if (c == 'w')
        {
            ROS_INFO("forward");
            Key_out3 = 1;
        }
        else if (c == 's')
        {
            ROS_INFO("backward");
            Key_out4 = 1;
        }

        keySignal.data.clear();
        keySignal.data.push_back(Key_out1);
        keySignal.data.push_back(Key_out2);
        keySignal.data.push_back(Key_out3);
        keySignal.data.push_back(Key_out4);
        outputKey.publish(keySignal);
        // wait
        ros::spinOnce();
        loop_rate.sleep();

    } // main loop -> ros::ok
    return 0;
}
