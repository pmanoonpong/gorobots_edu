

///////////////////////////////////////////
#include "Serial.h"
#include <string.h>
#include <stdio.h>
#define BPM_ITER	200000 
///////////////////////////////////////////

Serial* robot;


int main (int argc, char **argv)
{
 
	

robot = new Serial();	
int t;

/////Initial Neural network///////////////////////


for (int i=0;i<BPM_ITER;i++) {


//XNOR sigmoid function training//
robot->FeedforwardNetwork(0,0,1);	
robot->FeedforwardNetwork(0,1,0);
robot->FeedforwardNetwork(1,0,0);
robot->FeedforwardNetwork(1,1,1);


//XNOR function testing set //
/*
robot->Run(0,0);
robot->Run(0,1);
robot->Run(1,0);
robot->Run(1,1);
*/

}


  return 0;
}
