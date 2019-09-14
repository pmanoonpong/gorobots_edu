/*
 * Giuliano Di Canio 3 November 2014
 */
#ifndef DUNGBEETLESERIAL_H_
#define DUNGBEETLESERIAL_H_

#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <iterator>


#include <assert.h>
#include <stdlib.h>

#include <selforg/abstractrobot.h>
#include <selforg/matrix.h>

#include "cmdline.h"
#include "console.h"
#include "globaldata.h"

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // terminal control definitions
#include <time.h>   // time calls

#include<sys/time.h>

//#include "sensor_motor_definition.h"
#include <utils/real_robots/dungbeetle/dungBeetle_hindlegSensMotDef.h>

using namespace std;

#define COM1 "/dev/ttyS0"

#define SENSOR_BUFFER_NUM 34     //numbers for M-board 1
#define MOTOR_BUFFER_NUM 33      //numbers for M-board 1
#define COMMAND_BUFFER_NUM 100


namespace lpzrobots {


/** This class communicates with amosII robot
 */
class dungBeetleSerial : public AbstractRobot {
public:
	dungBeetleSerial(const char *port = "/dev/ttyS0");


	~dungBeetleSerial();

	// robot interface
	/** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1]
      @param sensornumber length of the sensor array
      @return number of actually written sensors
	 */
	virtual int getSensors(sensor* sensors, int sensornumber);

	/** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
	 */
	virtual void setMotors(const motor* motors, int motornumber);


	/** returns number of sensors */
	virtual int getSensorNumber(){ return sensornumber; }

	/** returns number of motors */
	virtual int getMotorNumber() { return motornumber; }

	/* the following are not used here, you can ignore them but keep them*/
	virtual Position getPosition()     const {return Position(0,0,0);}
	virtual Position getSpeed()        const {return Position(0,0,0);}
	virtual Position getAngularSpeed() const {return Position(0,0,0);}
	virtual matrix::Matrix getOrientation() const {
		matrix::Matrix m(3,3);
		m.toId();
		return m;
	};

	/*Default sensors processing*/
	virtual void processSensors(sensor* pSensor);
	/*Your own sensors processing*/
	virtual void processSensorsGiuliano(sensor* pSensor);



private:

	int fd1;// Return char after opening COM0
	double  Sensor[DUNGBEETLE_SENSOR_MAX]; //but only 18 sensors used now, others are set to zero
	double motorCom[DUNGBEETLE_MOTOR_MAX];
	char chBuff;

	int servoPosMin[DUNGBEETLE_MOTOR_MAX];
	int servoPosMax[DUNGBEETLE_MOTOR_MAX];

	int nSensorType;
	double potValue[SENSOR_BUFFER_NUM];

	int index;//=0;
	double sensor1;//=1;
	int wr, rd;
	char comByte;//=2;
	char end;//=0;

	struct termios port_settings;// structure to store the port settings in
	struct termios tio;
	struct termios stdio;

	char serial_motor[100];
	int serialPos[MOTOR_BUFFER_NUM];

	int motornumber;
	int sensornumber;

	const char * port;

	paramval power;
	paramval tempThesh;
	paramval accelFactor;
	int t;


	enum RealDungBeetleSensorNames{

	 /*Add more sensors here according to the port of the MBoard*/

	  // Foot contact sensors
	  TC0_RIGHT_REAL= 10,
	  CT0_RIGHT_REAL= 11,
	  FT0_RIGHT_REAL= 12,


	 };

};

}
#endif /* dungBeetleSerial_H_ */
