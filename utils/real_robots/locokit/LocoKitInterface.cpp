/*
 * LocoKitInterface.cpp
 *
 *  Created on: Mar 29, 2014
 *      Author: Bassel Zeidan
 */

#include "LocoKitInterface.h"
#define ROBOT_IP_ADDRESS "192.168.2.4"//"127.0.0.1"
#define PORT 1214



LocoKitInterface::LocoKitInterface() {
	connected = false;
	for (int i=0; i<6; i++)
		sensory_inputs[i] = 0;
}

LocoKitInterface::~LocoKitInterface() {
	receiving_connection.close_connection = true;
}

//establish the connection with the server side
//return 0 on success, -1 for errors
int LocoKitInterface::establish_connection() {
	sending_connection.define_client(ROBOT_IP_ADDRESS, PORT);
	if ((!sending_connection.connected)
			) {
		connected = false;
		printf("a connection couldn't be established...\n");
		printf("Terminate program...\n");
		return -1;
	}
	return 0;
}

//Set the PWM of an actuator.
//Parameters:
//pwm: The PWN in the range from -1024 to 1024
//Actuator: The ID of the actuator
//Return 0 if successful, -1 for errors.
int LocoKitInterface::setActuatorPWM(float pwm, int actuator) {
	int function_code = 2; //defined in the protocol
	char* temp =  (char *) &function_code;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &actuator;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &pwm;
	if (sending_connection.send_command(sizeof(float), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	return 0;
}

//Stop an actuator.
//Parameters:
//actuator: The ID of the actuator
//Returns: 	0 if successful, -1 for errors
int LocoKitInterface::setActuatorStopped(int actuator) {
	int function_code = 10; //defined in the protocol
	char* temp =  (char *) &function_code;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &actuator;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	return 0;
}

//Set the motor control interpolation values.
//Parameters:
//actuator:	The motor to control
//period: The period time in seconds
//phaseOffset: The phaseOffset in degrees
//directionNegative: The direction of rotation, 0 for rotation in positive direction 1 for negative
//Returns: 0 if successful, -1 for errors
int LocoKitInterface::setConstantSpeedInterpolatingFunction (int actuator, float period, float phaseOffset, int directionNegative) {
	int function_code = 3; //defined in the protocol
	char* temp =  (char *) &function_code;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &actuator;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &period;
	if (sending_connection.send_command(sizeof(float), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &phaseOffset;
	if (sending_connection.send_command(sizeof(float), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &directionNegative;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	return 0;
}

//This function is called to terminate the connection with the server. It send a command
//to the server to terminate the server program also
int LocoKitInterface::terminate_connection_with_server() {
	int function_code = 50; //defined in the protocol
	char* temp =  (char *) &function_code;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	return 0;
}

////returns the number of sensors
//returns: 0 if successful, -1 for errors
int LocoKitInterface::getNumberOfSensors() {
	int function_code = 4; //defined in the protocol
	char* temp =  (char *) &function_code;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	char buffer [sizeof(int)];
	char* c;
	c = buffer;
	if (sending_connection.receive_command(sizeof(int), &c) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	return *((int*)(c));
}

//get the current unmodified value of the sensor (returned in the "sensor_value" parameter)
//Parameters: The ID of the sensor
//returns: 0 if successful, -1 for errors
//sensors ID numbers:
//sensor (0) ---> acc on x
//sensor (1) ---> acc on y
//sensor (2) ---> acc on z
//sensor (3) ---> gyo x
//sensor (4) ---> gyo y
//sensor (5) ---> gyo z
int LocoKitInterface::getSensorValueRawFloat(int sensor, float &sensor_value) {
	int function_code = 5; //defined in the protocol
	char* temp =  (char *) &function_code;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &sensor;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}

	char buffer [sizeof(float)];
	char* c;
	c = buffer;
	if (sending_connection.receive_command(sizeof(float), &c) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	sensor_value = *(float*)c;
	return 1;
}

//Get the velocity of an actuator.
//Parameters:
//actuator: The ID of the actuator
//provide the velocity in degrees per second in the parameter  “velocity”
//returns 0 if successful, -1 otherwise
int LocoKitInterface::getActuatorVelocity(int actuator, float& velocity) {
	int function_code = 7; //defined in the protocol
	char* temp =  (char *) &function_code;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &actuator;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}

	char buffer [sizeof(float)];
	char* c;
	c = buffer;
	if (sending_connection.receive_command(sizeof(float), &c) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	velocity = *(float*)buffer;
	return 1;
}

//Get the current position of an actuator.
//Parameters:
//actuator: The ID of the actuator
//provide the position in degrees in the parameter  “position”
//returns 0 if successful, -1 otherwise
int LocoKitInterface::getActuatorPosition(int actuator, float& position) {
	int function_code = 8; //defined in the protocol
	char* temp =  (char *) &function_code;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &actuator;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}

	char buffer [sizeof(float)];
	char* c;
	c = buffer;
	if (sending_connection.receive_command(sizeof(float), &c) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	position = *(float*)buffer;
	return 1;
}

//get the PWM of an actuator
//Parameters:
//actuator: the ID of the actuator
//provides the PWM in the parameter "PMW_value"
//returns 0 if successful, -1 otherwise
int LocoKitInterface::getActuatorPWM(int actuator, float& PWM_value) {
	int function_code = 9; //defined in the protocol
	char* temp =  (char *) &function_code;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &actuator;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}

	char buffer [sizeof(float)];
	char* c;
	c = buffer;
	if (sending_connection.receive_command(sizeof(float), &c) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	PWM_value = *(float*)buffer;
	return 1;
}

//provides 6 values that are pointed out in a previous
//function (getSensorValueRawFloat ID:0..5) in one array and at the same order.
//these values are saved in the public member sensory_inputs.
//returns 0 if successful, -1 otherwise
//sensor (0) ---> acc on x
//sensor (1) ---> acc on y
//sensor (2) ---> acc on z
//sensor (3) ---> gyo x
//sensor (4) ---> gyo y
//sensor (5) ---> gyo z
int LocoKitInterface::updateSensorValueRawFloat_array() {
	int function_code = 6; //defined in the protocol
	char* temp =  (char *) &function_code;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	char buffer [sizeof(float)*6]; //6 float values
	char* c;
	c = buffer;
	if (sending_connection.receive_command(sizeof(float)*6, &c) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}

	float* f = (float*)buffer;
	for (int i=0; i<6; i++)
		sensory_inputs[i] = f[i];
	return 1;
}

//Get current sensor value.
//Parameters:
//sensor: the ID of the sensor
//provides the current value of the sensor in the parameter "sensor_value"
//returns 0 if successful, -1 otherwise
int LocoKitInterface::getSensorValue(int sensor, float &sensor_value) {
	int function_code = 11; //defined in the protocol
	char* temp =  (char *) &function_code;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &sensor;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}

	char buffer [sizeof(float)];
	char* c;
	c = buffer;
	if (sending_connection.receive_command(sizeof(float), &c) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	sensor_value = *(float*)c;
	return 1;
}

//reset a sensor. The current sensor value is used as offset for future
//calls to getSensorValue but not getSensorValueRaw.
//Parameters:
//sensor: the ID of the sensor
//returns 0 if successful, -1 otherwise
int LocoKitInterface::resetSensorValue(int sensor) {
	int function_code = 12; //defined in the protocol
	char* temp =  (char *) &function_code;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	temp =  (char *) &sensor;
	if (sending_connection.send_command(sizeof(int), temp) == -1) {
		printf("LocoKitInterface warning: a message has not been sent...\n");
		return -1;
	}
	return 0;
}
