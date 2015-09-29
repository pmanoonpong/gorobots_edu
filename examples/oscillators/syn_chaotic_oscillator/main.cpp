//en, G.; Chen, W.; Dasgupta, S.; Kolodziejski, C.; Wörgötter, F.; Manoonpong, P. (2015)Multiple Chaotic Central Pattern Generators with Learning for Legged Locomotion and Malfunction Compensation. Information Sciences 294, 666-682
///////Serial Communication////////////////
#include "Serial.h"
#include <string.h>
#include <stdio.h>
///////////////////////////////////////////


Serial* robot;

int main (int argc, char **argv)
{



robot = new Serial();	
/////End Initial parameters////////////////////


	do{


    //////////////Control Architecture////////////////////////////////////////////////////


	robot->ChaosCPG();

   //////////////End Control Architecture/////////////////////////////////////////////////


     
	}while(1==1);

  return 0;
}

