/*
 *
 * Written by Sun Tao 2019-9-9
 * 
 */
#include "rosClass.h"
#include <math.h>
using namespace stcontroller;

#define HipAmp 0.2
#define KneeAmp 0.15
#define StepFre 15.0
#define HipMNBias -0.2
#define KneeMNBias -0.2
int main(int argc,char* argv[])
{
    RosClass ros(argc,argv);
    std::vector<float> motorValue;
    motorValue.resize(12);
    unsigned long int t=0;
    while(ros::ok()){
        if(ros.terminate)
            return false;
        t=t+1;
        motorValue.at(0)=0.0;
        motorValue.at(1)=HipAmp*sin(t/StepFre*M_PI)+HipMNBias;
        motorValue.at(2)=KneeAmp*cos(t/StepFre*M_PI)+KneeMNBias;

        motorValue.at(3)=0.0;
        motorValue.at(4)=HipAmp*sin(t/StepFre*M_PI+M_PI)+HipMNBias;
        motorValue.at(5)=KneeAmp*cos(t/StepFre*M_PI+M_PI)+KneeMNBias;

        motorValue.at(6)=0.0;
        motorValue.at(7)=HipAmp*sin(t/StepFre*M_PI+M_PI)+HipMNBias;
        motorValue.at(8)=KneeAmp*cos(t/StepFre*M_PI+M_PI)+KneeMNBias;

        motorValue.at(9)=0.0;
        motorValue.at(10)=HipAmp*sin(t/StepFre*M_PI)+HipMNBias;
        motorValue.at(11)=KneeAmp*cos(t/StepFre*M_PI)+KneeMNBias;


        ros.setMotorValue(motorValue);
        ros.rosSleep();
    }

    return 0;
}
