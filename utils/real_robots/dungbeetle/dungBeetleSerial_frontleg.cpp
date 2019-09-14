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
#include <utils/real_robots/dungbeetle/dungBeetleSerial_frontleg.h>




namespace lpzrobots {

dungBeetleSerial::dungBeetleSerial(const char *port)
: AbstractRobot("dungBeetleSerial", "$Id: main.cpp,v 0.1 2011/14/07 18:00:00 fhesse $"),
  port(port) {

	fd1=open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);//make sure your account in PC can have access to serial port


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

		memset(&tio,0,sizeof(tio));
		tio.c_iflag=0;
		tio.c_oflag=0;
		tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
		tio.c_lflag=0;
		tio.c_cc[VMIN]=1;
		tio.c_cc[VTIME]=5;


		cfsetospeed(&tio,B57600);            // 57600 baud
		cfsetispeed(&tio,B57600);            // 57600 baud

		tcsetattr(fd1,TCSANOW,&tio);


		printf("port finish configuration \n");
	}
	comByte=2;
	end=0;
	index=0;
	sensor1=1;


	t=0;  // global step counter

	sensornumber = DUNGBEETLE_FRONTLEG_SENSOR_MAX;
	motornumber = DUNGBEETLE_FRONTLEG_MOTOR_MAX;

	//Setting Motors
	for (int t=0;t<33;t++)
	{
		serialPos[t]=128; //Setting all motor to middle position as initialization
	}

}


dungBeetleSerial::~dungBeetleSerial(){


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
int dungBeetleSerial::getSensors(sensor* sensors, int sensornumber){

	assert(sensornumber >= this->sensornumber);

	for(int i=0; i<=DUNGBEETLE_FRONTLEG_SENSOR_MAX;i++){
			sensors[i]=0;
	}

	comByte=2;
	end=0;

	char serial_msg[COMMAND_BUFFER_NUM];
	do{

		//Sending "getSensors" command to the board
		sprintf(serial_msg, "%c%c",comByte,end);

		wr = write(fd1, serial_msg,sizeof(serial_msg));

		// --- Reading the potentiometer values
		for (int i=1;i<SENSOR_BUFFER_NUM;i++){
			do{

				rd = read(fd1, &chBuff, 1);
				if (rd){
					potValue[i]=(unsigned char)(chBuff);// potvalue are AMOS sensor data
				}
			}while(!rd);

		}

	}while((unsigned char)(chBuff)!=0);// "0" is sync byte


	// LpzRobot <-- AMOS
	//Foot sensors (FS,Group 1)
	sensors[COXA1_RIGHT]=potValue[COXA1_RIGHT_REAL];
	sensors[COXA2_RIGHT]=potValue[COXA2_RIGHT_REAL];
	sensors[FEMUR_RIGHT]=potValue[FEMUR_RIGHT_REAL];
	sensors[TIBIA_RIGHT]=potValue[TIBIA_RIGHT_REAL];

	sensors[COXA1_LEFT]=potValue[COXA1_LEFT_REAL];
	sensors[COXA2_LEFT]=potValue[COXA2_LEFT_REAL];
	sensors[FEMUR_LEFT]=potValue[FEMUR_LEFT_REAL];
	sensors[TIBIA_LEFT]=potValue[TIBIA_LEFT_REAL];


	//Conversion to positive range [0,..,255]
	for(int i=0; i<=DUNGBEETLE_FRONTLEG_SENSOR_MAX;i++){
		if (sensors[i] < 0){
			sensors[i]+=256;
		}
	}


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
void dungBeetleSerial::processSensors(sensor* psensors){


	//Need to do proper mapping into proper range, at the moment these values are between 0...255
	psensors[COXA1_RIGHT] = psensors[COXA1_RIGHT]+0;
	psensors[COXA2_RIGHT] = psensors[COXA2_RIGHT]+0;
	psensors[FEMUR_RIGHT] = psensors[FEMUR_RIGHT]+0;
	psensors[TIBIA_RIGHT] = psensors[TIBIA_RIGHT]+0;

	psensors[COXA1_LEFT] = psensors[COXA1_LEFT]+0;
	psensors[COXA2_LEFT] = psensors[COXA2_LEFT]+0;
	psensors[FEMUR_LEFT] = psensors[FEMUR_LEFT]+0;
	psensors[TIBIA_LEFT] = psensors[TIBIA_LEFT]+0;

}




/** sets actual motorcommands
  @param motors motors scaled to [-1,1]
  @param motornumber length of the motor array
 */
void dungBeetleSerial::setMotors(const motor* motors, int motornumber){

	assert(motornumber >= this->motornumber);


	end =0; // Null as Sync-byte
	comByte=1;


	// -------------------- initializing the Motor range ------------------------

	//[-45,.., 45 deg]
	//Coxa1_right
	servoPosMin[0] = 30;
	servoPosMax[0] = 140;
	//Coxa2_right
	servoPosMin[1] = 150;
	servoPosMax[1] = 80;
	//Femur_right
	servoPosMin[2] = 120;
	servoPosMax[2] = 230;
	//Tibia_right
	servoPosMin[3] = 40;
	servoPosMax[3] = 240;

	//Coxa1_left
	servoPosMin[4] = 200;
	servoPosMax[4] = 50;
	//Coxa2_left
	servoPosMin[5] = 160;
	servoPosMax[5] = 250;
	//Femur_left
	servoPosMin[6] = 170;
	servoPosMax[6] = 50;
	//Tibia_left
	servoPosMin[7] = 210;//160;
	servoPosMax[7] = 40;//80;

	//Body
	servoPosMin[8] = 240;
	servoPosMax[8] = 40;

	// ##################### move motors ################
	for(int i=0;i<DUNGBEETLE_FRONTLEG_MOTOR_MAX;i++)
	{
		motorCom[i] =  motors[i];// set LpzMotor value before processing and sending ??????????what is this?

	  if (motorCom[i]>1) motorCom[i]=1;
	  if (motorCom[i]<-1) motorCom[i]=-1;
	}

  servoPosMin[0] = 30;
  servoPosMax[0] = 140;

	//motorCom[5] = 1;

	//COXA1_RIGHT-J1
	serialPos[32] = (int) (double)(((motorCom[0]+1.0)/2.0)*(servoPosMax[0]-servoPosMin[0])+servoPosMin[0]) ;
	//COXA2_RIGHT-J2
	serialPos[31] = (int) (double)(((motorCom[1]+1.0)/2.0)*(servoPosMax[1]-servoPosMin[1])+servoPosMin[1]) ;
	//FEMUR_RIGHT-J3
	serialPos[30] = (int) (double)(((motorCom[2]+1.0)/2.0)*(servoPosMax[2]-servoPosMin[2])+servoPosMin[2]) ;
	//TIBIA_RIGHT-J4
	serialPos[29] = (int) (double)(((motorCom[3]+1.0)/2.0)*(servoPosMax[3]-servoPosMin[3])+servoPosMin[3]) ;


	//COXA1_LEFT-J1
	serialPos[28] = (int) (double)(((motorCom[4]+1.0)/2.0)*(servoPosMax[4]-servoPosMin[4])+servoPosMin[4]) ;
	//COXA2_LEFT-J2
	serialPos[27] = (int) (double)(((motorCom[5]+1.0)/2.0)*(servoPosMax[5]-servoPosMin[5])+servoPosMin[5]) ;
	//FEMUR_LEFT-J3
	serialPos[26] = (int) (double)(((motorCom[6]+1.0)/2.0)*(servoPosMax[6]-servoPosMin[6])+servoPosMin[6]) ;
	//TIBIA_LEFT-J4
	serialPos[25] =  (int) (double)(((motorCom[7]+1.0)/2.0)*(servoPosMax[7]-servoPosMin[7])+servoPosMin[7]) ;


	//Body-J1
	serialPos[24] = (int) (double)(((motorCom[8]+1.0)/2.0)*(servoPosMax[8]-servoPosMin[8])+servoPosMin[8]) ;



	//usleep(1000);
	//usleep (10000);//10000);
	// do some processing for motor commands before sending AMOS sensors

	sprintf(serial_motor, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
				,comByte,serialPos[1],serialPos[2],
				serialPos[3],serialPos[4],serialPos[5],serialPos[6],serialPos[7],serialPos[8],
				serialPos[9],serialPos[10],serialPos[11],serialPos[12],serialPos[13],serialPos[14],
				serialPos[15],serialPos[16],serialPos[17],serialPos[18],serialPos[19],serialPos[20],
				serialPos[21],serialPos[22],serialPos[23],serialPos[24],serialPos[25],serialPos[26],
				serialPos[27],serialPos[28],serialPos[29],serialPos[30],serialPos[31],serialPos[32],end);

	//Sendding command to serial port
	write(fd1, serial_motor, 34);//sizeof(serial_msg));

	// to slow down process a bit
	//usleep(10000);

	usleep (100000);//10000);

	// increase time counter
	t++;


}

/*Process your sensor signals here to match to your need*/
void dungBeetleSerial::processSensorsGiuliano(sensor* sensors){

}


}




