#include <iostream>
#include <ros/ros.h>
#include "robot.h"
using namespace lilibot_ns;

int main(int argc, char** argv){
    Robot robot(argc, argv);
    while(robot.run()){
        
        }
    return 0;

}
