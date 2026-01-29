//Steingrube, S.; Timme, M.; Woergoetter, F.; Manoonpong, P. (2010) Self-Organized Adaptation of Simple Neural Circuits Enables Complex Robot Behavior. Nature Physics 6, 224-230.


///////Serial Communication////////////////
#include "Serial.h"
#include <string.h>
#include <stdio.h>
///////////////////////////////////////////


Serial* robot;

int main (int argc, char **argv)
{


/////define parameters///////////////////////
int counter;


robot = new Serial();	
/////End Initial parameters////////////////////


	do{


	/////////////////////////////////////////



    //////////////Control Architecture////////////////////////////////////////////////////


	robot->ChaosCPG();

   //////////////End Control Architecture/////////////////////////////////////////////////


    //printf( "Plusdifff milliseconds:\t\t\t%u\n",difftime);
		
    //Or check Update freq by recoard data in one min = number of data/60 = Hz

	}while(1==1);

  return 0;
}

