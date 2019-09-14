/***************************************************************************
 *   Copyright (C) 2012 by Robot Group Goettingen                          *
 *                                    									   *
 *    fhesse@physik3.gwdg.de     			                               *
 *    xiong@physik3.gwdg.de                  	                           *
 *    poramate@physik3.gwdg.de                                             *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   AMOSII v2 has now only 18 sensors                                     *
 ***************************************************************************/

// include header file
#include <utils/real_robots/dacbot/dacbot_serial.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>

unsigned int triang = 0;
bool sw = true;
int abc=0;
int feedback[4];

int cntr=0;

int motcom[4];
int hip_right;
int hip_left;
int knee_right;
int knee_left;

namespace lpzrobots {

dacbot_serial::dacbot_serial(const char *port)
: AbstractRobot("dacbot_serial", "$Id: main.cpp,v 0.1 2011/14/07 18:00:00 fhesse $"),
  port(port) {

	std::cout<<"Opening serial port"<<std::endl;
	fd1=open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);//make sure your account in PC can have access to serial port


	if(fd1 == -1)
	{
		std::cout<<std::endl<<"unable to open"<<port<<std::endl;
		std::cout<<"check provided port (hint: \"/dev/ttyS0\" or \"/dev/ttyUSB0\" required?)"<<std::endl;
		assert (fd1 != -1);
	}
	else
	{
		fcntl(fd1, F_SETFL, 0);
		printf("port is open");
		std::cout<<"Serial port open"<<std::endl;

		memset(&tio,0,sizeof(tio));
		tio.c_iflag=0;
		tio.c_oflag=0;
		tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
		tio.c_lflag=0;
		tio.c_cc[VMIN]=1;
		tio.c_cc[VTIME]=5;


		cfsetospeed(&tio,B115200);            // 115200 baud
		cfsetispeed(&tio,B115200);            // 115200 baud

		tcsetattr(fd1,TCSANOW,&tio);


		printf("port finish configuration \n");
	}
	comByte=2;
	end=0;
	index=0;
	sensor1=1;


	t=0;  // global step counter

	//Variables in Ducbot_hindlegSensMotDef.h
	sensornumber = DACBOT_SENSOR_MAX;
	motornumber = DACBOT_MOTOR_MAX;

	//Setting Motors
	for (int t=0;t<33;t++)
	{
		serialPos[t]=128; //Setting all motor to middle position as initialization
	}

	serialPlot.open("serialplot.dat");

}


dacbot_serial::~dacbot_serial(){


	//Close COM0
	if(fd1 != -1){
		close(fd1);
		std::cout<<"closed the serial communication port"<<std::endl;
	}
}


// robot interface
/** returns actual sensorvalues
  @param sensors sensors scaled to [-1,1]
  @param sensornumber length of the sensor array
  @return number of actually written sensors
 */
int dacbot_serial::getSensors(sensor* sensors, int sensornumber){

	assert(sensornumber >= this->sensornumber);

	for(int i=0; i<=DACBOT_SENSOR_MAX;i++){
		sensors[i]=0;
	}

	char serial_msg[2]= {2,0};
	wr = write(fd1, serial_msg,sizeof(serial_msg));
	//std::cout<<"Request sent: "<<wr<<std::endl;

	//sleep(2);
	do{

		// --- Reading the potentiometer values
		for (int i=0;i<SENSOR_BUFFER_NUM;i++){
			do{
				rd = read(fd1, &chBuff, 1);
				if (rd){
					potValue[i]=(unsigned char)(chBuff);// potvalue are AMOS sensor data
					//std::cout<<"FB "<< i << ":" <<potValue[i]<<std::endl;
					if(chBuff==0){
						received=true;
						//break;
					}
				}
			}while(!rd);
		}
	}while(!received);// "0" is sync byte

	received=false;

	//Foot sensors (FS,Group 1)
	sensors[0]=potValue[0]; //HL
	sensors[1]=potValue[1]; //HR
	sensors[2]=potValue[2];	//KL
	sensors[3]=potValue[3]; //KR
	sensors[4]=0.5;			//BA
	potValue[4]-=1;
	potValue[5]-=1;
	sensors[5]=((-potValue[4]+1)+1)*2047;//FSL
	sensors[6]=((-potValue[5]+1)+1)*2047;//FSR

	/*std::cout<<"sensors 1 : "<<sensors[0]<<std::endl;
   	std::cout<<"sensors 2 : "<<sensors[1]<<std::endl;
	std::cout<<"sensors 3 : "<<sensors[2]<<std::endl;
   	std::cout<<"sensors 4 : "<<sensors[3]<<std::endl;
	std::cout<<"sensors 5 : "<<sensors[5]<<std::endl;
   	std::cout<<"sensors 6 : "<<sensors[6]<<std::endl;*/


	bool default_preprocessing = true;//true
	if (default_preprocessing){
		processSensors(sensors);
	}

	//Your own,e.g.,
	bool giuliano_preprocessing = false;
	if (giuliano_preprocessing){
		processSensorsGiuliano(sensors);
	}


	return this->sensornumber;

}

/*Different sensors processing*/////// THIS ONE HAS TO BE SET UP BY SKRETCH
void dacbot_serial::processSensors(sensor* psensors){


	psensors[0]= ((psensors[0]-8)/(155-8))*(135-1)+1;
	psensors[1]= ((psensors[1]-8)/(155-8))*(135-1)+1;

	if(psensors[0] < 0)
	{
		psensors[0]=0;
	}

	if(psensors[1] < 0)
	{
		psensors[1]=0;
	}

	psensors[2]= ((psensors[2]-82)/(245-82))*(180-65)+65;
	psensors[3]= ((psensors[3]-72)/(235-72))*(180-65)+65;

	for (int i=0; i<4; i++) {
		if (psensors[i] < 0) {
			psensors[i]=0;
		}
	}


	/*std::cout<<"RA HL:"<< psensors[0]<<std::endl;
	std::cout<<"RA HR:"<< psensors[1]<<std::endl;
	std::cout<<"RA KL:"<< psensors[2]<<std::endl;
	std::cout<<"RA KR:"<< psensors[3]<<std::endl;
	std::cout<<"RA LFS:"<< psensors[5]<<std::endl100;
	std::cout<<"RA RFS:"<< psensors[6]<<std::endl;*/



}


/** sets actual motorcommands
  @param motors motors scaled to [-1,1]
  @param motornumber length of the motor array
 */

void dacbot_serial::setMotors(const motor* motors, int motornumber){

	assert(motornumber >= this->motornumber);


	// ##################### move motors ################
 	//motcom[0] = Left hip, 4 = + Voltage(move forward), 2 = - Voltage(move backward), 3 = 0 Voltage (not move) 
 	//motcom[1] = Right hip, 4 = + Voltage(move forward), 2 = - Voltage(move backward), 3 = 0 Voltage (not move) 
 	//motcom[2] = Right knee, 4 = + Voltage(move forward), 2 = - Voltage(move backward), 3 = 0 Voltage (not move)
 	//motcom[3] = Left knee, 4 = + Voltage(move forward), 2 = - Voltage(move backward), 3 = 0 Voltage (not move)

	// This arduino controller is on/off control cannot regulate voltage! It always provides MAX voltage!
	// Amplitude of the outputs of the DACbot controller does not effect the voltage, Only the duration is matter
	// The duration defines how long should move a leg forward/backward!!! 

	for(int i=0;i<4;i++)
	{
		motorCom[i] = motors[i];

		if (motorCom[i]>=0.5) {
			motcom[i]=4; // + Voltage
		} else if (motorCom[i]<=-0.5) {
			motcom[i]=2; // - Voltage
		} else {
			motcom[i]=3; // 0 Voltage
		}
	}

	//Transmit to Arduino controller
	hip_left = motcom[0];
	hip_right = motcom[1];
	knee_left = motcom[2];
	knee_right = motcom[3];
	



	serialPlot<<motcom[0]<<' '<<endl;
	/*std::cout<<"MC 1:"<< motcom[0]<<std::endl;*/

	//*******************************
	/*
//state 1
if (feedback[0]<= 90 && feedback[1]>= 90 && feedback[2]>= 170 && feedback[3]>= 170)  {
	motcom[0]= 1; //LH
	motcom[1]= 3; //RH
	motcom[2]= 1; //LK
	motcom[3]= 1; //RK
}

//state 2
if (feedback[0]<= 90 && feedback[1]>= 120 && feedback[2]>= 170 && feedback[3]>= 170){
	motcom[0]= 3; //LH
	motcom[1]= 1; //RH
	motcom[2]= 1; //LK
	motcom[3]= 1; //RK
}

//state 3
if (feedback[0]>= 120 && feedback[1]<= 90 && feedback[2]<= 90 && feedback[3]>= 170){
	motcom[0]= 1; //LH
	motcom[1]= 1; //RH
	motcom[2]= 3; //LK
	motcom[3]= 1; //RK
}

//state 4
if (feedback[0]<= 90 && feedback[1]<= 60 && feedback[2]>= 170 && feedback[3]<= 100){
	motcom[0]= 1; //LH
	motcom[1]= 3; //RH
	motcom[2]= 1; //LK
	motcom[3]= 3; //RK
}
	 */
	//*******************************


	// do some processing for motor commands before sending AMOS sensors
	//char serial_motor2[34] = {1,motcom[0],motcom[1],motcom[2],motcom[3],121,121,121,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,5,5,5,5,5,5,5,5,5,5,0};
	char serial_motor2[34] = {1,hip_left,hip_right,knee_right,knee_left,121,121,121,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,5,5,5,5,5,5,5,5,5,5,0};
	
	//Sendding command to serial port
	int n = write(fd1, serial_motor2, sizeof(serial_motor2));
	//usleep (10000);
	// increase time counter
	t++;


}

/*Process your sensor signals here to match to your need*/
void dacbot_serial::processSensorsGiuliano(sensor* sensors){

}


}
