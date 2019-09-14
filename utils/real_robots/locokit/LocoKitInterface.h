/*
 * LocoKitInterface.h
 *
 *  Created on: Mar 29, 2014
 *      Author: Bassel Zeidan
 *
 */

#include <pthread.h>
#include "ConnectionClass.h"


class LocoKitInterface {
private:
	ConnectionClass sending_connection;
	ConnectionClass receiving_connection;
	pthread_t receiving_thread;
	bool connected;

public:
	float sensory_inputs [6];

	LocoKitInterface();
	~LocoKitInterface();

	//establish the connection with the server side
	//return 0 on success, -1 for errors
	int establish_connection();

	//Set the PWM of an actuator.
	//Parameters:
	//pwm: The PWN in the range from -1024 to 1024
	//Actuator: The ID of the actuator
	//Return 0 if successful, -1 for errors.
	int setActuatorPWM(float pwm, int actuator);

	//Stop an actuator.
	//Parameters:
	//actuator: The ID of the actuator
	//Returns: 	0 if successful, -1 for errors
	int setActuatorStopped(int actuator);

	//Set the motor control interpolation values.
	//Parameters:
	//actuator:	The motor to control
	//period: The period time in seconds
	//phaseOffset: The phaseOffset in degrees
	//directionNegative: The direction of rotation, 0 for rotation in positive direction 1 for negative
	//Returns: 0 if successful, -1 for errors
	int setConstantSpeedInterpolatingFunction (int actuator, float period, float phaseOffset, int directionNegative);

	//////////////sensors///////////////////

	////returns the number of sensors
	//returns: 0 if successful, -1 for errors
	int getNumberOfSensors();

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
	int getSensorValueRawFloat(int sensor, float &sensor_value);

	//provides 6 values that are pointed out in the previous
	//function (getSensorValueRawFloat ID:0..5) in one array and at the same order.
	//these values are saved in the public member sensory_inputs.
	//returns 0 if successful, -1 otherwise
	int updateSensorValueRawFloat_array();

	//Get the velocity of an actuator.
	//Parameters:
	//actuator: The ID of the actuator
	//provide the velocity in degrees per second in the parameter  “velocity”
	//returns 0 if successful, -1 otherwise
	int getActuatorVelocity(int actuator, float& velocity);


	//Get the current position of an actuator.
	//Parameters:
	//actuator: The ID of the actuator
	//provide the position in degrees in the parameter  “position”
	//returns 0 if successful, -1 otherwise
	int getActuatorPosition(int actuator, float& position);

	//get the PWM of an actuator
	//Parameters:
	//actuator: the ID of the actuator
	//provides the PWM in the parameter "PMW_value"
	//returns 0 if successful, -1 otherwise
	int getActuatorPWM(int actuator, float& PWM_value);

	//Get current sensor value.
	//Parameters:
	//sensor: the ID of the sensor
	//provides the current value of the sensor in the parameter "sensor_value"
	//returns 0 if successful, -1 otherwise
	int getSensorValue(int sensor, float &sensor_value);

	//reset a sensor. The current sensor value is used as offset for future
	//calls to getSensorValue but not getSensorValueRaw.
	//Parameters:
	//sensor: the ID of the sensor
	//returns 0 if successful, -1 otherwise
	int resetSensorValue(int sensor);

	///////////////////////////////////////////////////////////////////

	//This function is called to terminate the connection with the server. It send a command
	//to the server to terminate the server program also
	int terminate_connection_with_server();
};







