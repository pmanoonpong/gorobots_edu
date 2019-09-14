
///////////////////////////////////////
//				server Side
///////////////////////////////////////
//			Auther: Bassel Zeidan
///////////////////////////////////////
//This code opens the connection
//with the client side (a computer which controls the robot) and waits for the
//commands to operate.

#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "locoapi.h"

#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>



#define PORT 1214 //connection port
#define false 0
#define true 1


struct sockaddr_in dest; /* socket info about the machine connecting to us */
struct sockaddr_in serv; /* socket info about our side */
int mysocket;            /* socket used to listen for incoming connections */
socklen_t socksize;
int connected;
int termination_code;
int receiving_socket;
int sending_socket;
int print_commands;


int enable_command_printing;


/////////////////////////////////////////////////////////////////
/////////////////////Connection Functions////////////////////////
/////////////////////////////////////////////////////////////////
void define_server(short int port_number) {
	enable_command_printing = false;
	memset(&serv, 0, sizeof(serv));           /* zero the struct before filling the fields */
	serv.sin_family = AF_INET;                /* set the type of connection to TCP/IP */
	serv.sin_addr.s_addr = htonl(INADDR_ANY); /* set our address to any interface */
	serv.sin_port = htons(port_number);           /* set the server port number */
	mysocket = socket(AF_INET, SOCK_STREAM, 0);//use tcp protocol
	/* bind serv information to mysocket */
	bind(mysocket, (struct sockaddr *)&serv, sizeof(struct sockaddr));
	connected = true;
	if (listen(mysocket, 1) == -1) {
		connected = false;
		printf("Error: listen(), can't listen on the socket...\n");
	}
}

int accept_a_client() { //returns client socket
	int connected_socket_descripter = accept(mysocket, (struct sockaddr *)&dest, &socksize);
	if (connected_socket_descripter == -1) {
		connected = false;
		printf("Error: accept(), can't connect to the socket and establish the connection...\n");
	}
	return connected_socket_descripter;
}

int send_command(int command, int client_socket) {
	char* message;
	message = (char *) &command;
	int rec_res;
	rec_res = send(client_socket, message, sizeof(int), 0);
	if (rec_res == -1) {
		printf("Warning: send(), a message hasn't been sent...\n");
		return -1;
	}
	return 0;
}

/////////////////////////////////////////////////////////////////
///////////////////////////Control routines//////////////////////
/////////////////////////////////////////////////////////////////
//applying the pwm value on the targeted actuator
//takes the actuator ID as an input in addition to the pwm value
void PWM_control_routine(int act_number, float pwm) {
	//Sets the controller mode
	setRegisterValueUI("CONTROLLER_MODE", act_number, 3);

	//Set the pmw value
	setActuatorPWM(pwm, act_number);
}

//applying the ConstantSpeedInterpolatingFunction
//values on the targeted actuator.
//it sets the motor control interpolation values.
//Parameters:
//actuator: The motor to control
//period: The period time in seconds
//phaseOffset: The phaseOffset in degrees
//directionNegative: the direction of rotation, 0 for rotation in positive direction 1 for negative
void ConstantSpeedInterpolatingFunction_control_routine(int act_number, float period,
		float phaseoffset, int direction) {
	//Sets the controller mode of the legs
	setRegisterValueUI("CONTROLLER_MODE", act_number, 5);

	//Set the speed and sync of the legs (even numbers)
	setConstantSpeedInterpolatingFunction(act_number, period, phaseoffset, direction);
}

//stop a targeted actuator. Takes the actuator number as an input
void setActuatorStopped_routine(int act_number) {
	setActuatorStopped(act_number);
}

//receives  setActuatorPWM parameters from the client and calls the required functions
//to execute the command.
//It takes the connected socket numebr as an input
//return 1 if successful, -1 errors, 0 connection is closed
int setActuatorPWM_command(int* socket) {
	char buffer_int[sizeof(int)];
	char buffer_float[sizeof(float)];
	int rec_res;
	int act_number;
	float pwm;
	//get actuator number
	rec_res = recv(*socket, buffer_int, sizeof(int), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	act_number = *((int *)buffer_int);
	//get pwm
	rec_res = recv(*socket, buffer_float, sizeof(float), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	pwm = *((float *)buffer_float);
	if (enable_command_printing == true)
		printf("PWM command, Act = %i, PWM = %f\n", act_number, pwm);
	PWM_control_routine(act_number, pwm);
	return 1;
}

//receives  setConstantSpeedInterpolatingFunction parameters
//from the client and calls the required functions to
//execute the command (It calls (ConstantSpeedInterpolatingFunction_control_routine).
//It takes the connected socket numebr as an input
//return 1 if successful, -1 errors, 0 connection is closed
int setConstantSpeedInterpolatingFunction_command(int* socket) {
	char buffer_int[sizeof(int)];
	char buffer_float[sizeof(float)];
	int rec_res;
	int act_number, direction;
	float period, phaseoffset;
	//get actuator number
	rec_res = recv(*socket, buffer_int, sizeof(int), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	act_number = *((int *)buffer_int);
	//get period
	rec_res = recv(*socket, buffer_float, sizeof(float), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	period = *((float *)buffer_float);
	//get phaseoffset
	rec_res = recv(*socket, buffer_float, sizeof(float), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	phaseoffset = *((float *)buffer_float);
	//get direction
	rec_res = recv(*socket, buffer_int, sizeof(int), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	direction = *((int *)buffer_int);
	if (enable_command_printing == true)
printf("ConstantSpeedInterpolatingFunction command, act = %i, period = %f, phaseoffset = %f,  direction = %i\n", act_number, period, phaseoffset, direction);
	ConstantSpeedInterpolatingFunction_control_routine(act_number, period, phaseoffset, direction);
	return 1;
}

//receives  the targeted motor ID from the client and
//calls the required functions to execute the command
//(stop command for a specific motor).
//It takes the connected socket numebr as an input
//return 1 if successful, -1 errors, 0 connection is closed
int setActuatorStopped_command(int* socket) {
	char buffer_int[sizeof(int)];
	int rec_res;
	int act_number;
	//get actuator number
	rec_res = recv(*socket, buffer_int, sizeof(int), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	act_number = *((int *)buffer_int);
	//call pwm function//////////////////////////////////////////////////here
	if (enable_command_printing == true)
		printf("stop Actuator number %i\n", act_number);
	setActuatorStopped_routine(act_number);
	return 1;
}

//calls the required functions to get the number of sensors
//It takes the connected socket numebr as an input
//return 1 if successful, -1 errors, 0 connection is closed
int getNumberOfSensors_procedure(int* socket) {
	int num_sensors = getNumberOfSensors();
	//printf("Number of sensors: \n", num_sensors);
	char* message;
	message = (char *) &num_sensors;
	int rec_res;
	rec_res = send(*socket, message, sizeof(int), 0);
	if (rec_res == -1) {
		printf("Warning: send(), a message hasn't been sent...\n");
		return -1;
	}
	return 1;
}

//receives  the targeted sensor ID from the client and
//calls the required functions to execute the function
//"getSensorValueRawFloat". It sends back the results to the client
//It takes the connected socket numebr as an input
//return 1 if successful, -1 errors, 0 connection is closed
int getSensorValueRawFloat_procedure(int* socket) {
	//get sensor number
	char buffer_int[sizeof(int)];
	int rec_res;
	int sensor_number;
	rec_res = recv(*socket, buffer_int, sizeof(int), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	sensor_number = *((int *)buffer_int);
	char* message;
	float value = getSensorValueRawFloat(sensor_number);
	message = (char *) &value;
	rec_res = send(*socket, message, sizeof(float), 0);
	if (rec_res == -1) {
		printf("Warning: send(), a message hasn't been sent...\n");
		return -1;
	}
	return 1;
}

//receives  the targeted sensor ID from the client and
//executes the function "getSensorValue" which get a sensor value.
//It sends back the results to the client
//It takes the connected socket numebr as an input
//return 1 if successful, -1 errors, 0 connection is closed
int getSensorValue_procedure(int* socket) {
	//get sensor number
	char buffer_int[sizeof(int)];
	int rec_res;
	int sensor_number;
	rec_res = recv(*socket, buffer_int, sizeof(int), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	sensor_number = *((int *)buffer_int);
	char* message;
	float value = getSensorValue(sensor_number);
	message = (char *) &value;
	rec_res = send(*socket, message, sizeof(float), 0);
	if (rec_res == -1) {
		printf("Warning: send(), a message hasn't been sent...\n");
		return -1;
	}
	return 1;
}

//receives  the targeted sensor ID from the client and
//executes the function "resetSensorValue" which reset a sensor.
//It sends back the results to the client
//It takes the connected socket numebr as an input
//return 1 if successful, -1 errors, 0 connection is closed
int resetSensorValue_procedure(int* socket) {
	//get sensor number
	char buffer_int[sizeof(int)];
	int rec_res;
	int sensor_number;
	rec_res = recv(*socket, buffer_int, sizeof(int), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	sensor_number = *((int *)buffer_int);
	resetSensorValue(sensor_number);
	return 1;
}

//executes the function "getSensorValueRawFloat" for all available sensors
//to get all sensors' values.
//It sends back the results to the client as an array of 6 elements
//sensor (0) ---> acc on x
//sensor (1) ---> acc on y
//sensor (2) ---> acc on z
//sensor (3) ---> gyo x
//sensor (4) ---> gyo y
//sensor (5) ---> gyo z
//It takes the connected socket numebr as an input
//return 1 if successful, -1 errors, 0 connection is closed
int getSensorValueRawFloat_array_procedure(int* socket) {
	//get sensor number
	int rec_res;
	char* message;
	float send_buffer[6];
	int i;
	for (i=0; i<6;i++)
		send_buffer[i] = getSensorValueRawFloat(i);

	message = (char *) send_buffer;
	rec_res = send(*socket, message, sizeof(float)*6, 0);
	if (rec_res == -1) {
		printf("Warning: send(), a message hasn't been sent...\n");
		return -1;
	}
	return 1;
}

//receives  the targeted motor ID from the client and
//executes the function "getActuatorVelocity" which gives the motor velocity.
//It sends back the results to the client
//It takes the connected socket numebr as an input
//return 1 if successful, -1 errors, 0 connection is closed
int getActuatorVelocity_procedure(int* socket) {
	//get sensor number
	char buffer_int[sizeof(int)];
	int rec_res;
	int sensor_number;
	rec_res = recv(*socket, buffer_int, sizeof(int), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	sensor_number = *((int *)buffer_int);
	char* message;
	float value = getActuatorVelocity(sensor_number);
	message = (char *) &value;
	rec_res = send(*socket, message, sizeof(float), 0);
	if (rec_res == -1) {
		printf("Warning: send(), a message hasn't been sent...\n");
		return -1;
	}
	return 1;
}

//receives  the targeted motor ID from the client and
//executes the function "getActuatorPosition" which gives the motor position.
//It sends back the results to the client
//It takes the connected socket numebr as an input
//return 1 if successful, -1 errors, 0 connection is closed
int getActuatorPosition_procedure(int* socket) {
	//get sensor number
	char buffer_int[sizeof(int)];
	int rec_res;
	int sensor_number;
	rec_res = recv(*socket, buffer_int, sizeof(int), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	sensor_number = *((int *)buffer_int);
	char* message;
	float value = getActuatorPosition(sensor_number);
	message = (char *) &value;
	rec_res = send(*socket, message, sizeof(float), 0);
	if (rec_res == -1) {
		printf("Warning: send(), a message hasn't been sent...\n");
		return -1;
	}
	return 1;
}

//receives  the targeted motor ID from the client and
//executes the function "getActuatorPWM" which gives the motor PMW.
//It sends back the results to the client
//It takes the connected socket numebr as an input
//return 1 if successful, -1 errors, 0 connection is closed
int getActuatorPWM_procedure(int* socket) {
	//get sensor number
	char buffer_int[sizeof(int)];
	int rec_res;
	int sensor_number;
	rec_res = recv(*socket, buffer_int, sizeof(int), 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	sensor_number = *((int *)buffer_int);
	char* message;
	float value = getActuatorPWM(sensor_number);
	message = (char *) &value;
	rec_res = send(*socket, message, sizeof(float), 0);
	if (rec_res == -1) {
		printf("Warning: send(), a message hasn't been sent...\n");
		return -1;
	}
	return 1;
}

//this function represent the protocol interpreter which
//interprete the commands that are coming from client.
//It takes the command number as an input and the client socket
//2 ----> PWM control
//3 ----> ConstantSpeedInterpolatingFunction control
//4 ----> get Number Of Sensors
//5 ----> get Sensor Value RawFloat
//6 ----> get Sensor Value RawFloat_array (returns an array)
//7 ----> get Actuator Velocity
//8 ----> get Actuator Position
//9 ----> get Actuator PWM
//10 ----> stop an Actuator
//11 ----> get Sensor Value
//12 ----> reset Sensor Value
//50 ----> termination code
int command_interpreter(int command, int* socket) {
	int res = -1;
	switch(command) {
		case 2: //PWM control
			res = setActuatorPWM_command(socket);
		break;
		case 3:
			res = setConstantSpeedInterpolatingFunction_command(socket);
		break;
		case 4:
			res = getNumberOfSensors_procedure(socket);
		break;
		case 5:
			res = getSensorValueRawFloat_procedure(socket);
			//sensor_value
		break;
		case 6:
			res = getSensorValueRawFloat_array_procedure(socket);
			//sensory_array
		break;
		case 7:
			res = getActuatorVelocity_procedure(socket);
			//velocity
		break;
		case 8:
			res = getActuatorPosition_procedure(socket);
		break;
		case 9:
			res = getActuatorPWM_procedure(socket);
		break;
		case 10:
			res = setActuatorStopped_command(socket);
		break;
		case 11:
			res = getSensorValue_procedure(socket);
		break;
		case 12:
			res = resetSensorValue_procedure(socket);
		break;
		case 50://termination code
			termination_code = true;
		break;
		default: //res = -1;
			return -1; //quit code
		break;
	}
	return res;
}



/////////////////////////End Control Procedures//////////////////////
int receive_command(int* command, int* client_socket) {
	char buffer[sizeof(int)];
	int rec_res;
	rec_res = recv(*client_socket, buffer, sizeof(int), 0);
	if (rec_res > 0) {
		*command = *((int *)buffer);
		return command_interpreter(*command, client_socket);
	} else if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	return rec_res;
}

void receive_commands() {
	int command;
	while (receive_command(&command, &receiving_socket) > 0) {
		//printf("received command = %i\n", command);
	}
}

int main(int argc, char *argv[])
{
	//initialize LocoAPI
	if(initializeLocoAPI( "loco-settings.cfg" ) != 0 ) {
		printf("Failed to initialize LocoAPI\n");
		exit(-1);
	}

	termination_code = false;
	print_commands = false;

	socksize = sizeof(struct sockaddr_in);
	printf("Listening...\n");
	define_server(PORT);
	while (termination_code == 0) {
		printf("waiting for new socket...\n");
		receiving_socket = accept_a_client();//wait for the first socket (receiving socket)
		printf("receiving connection is connected...\n");
		//sending_socket = accept_a_client();//wait for the first socket (receiving socket)
		//printf("sending connection is connected...\n");
		printf("a connection is established successfully with the w-lan controller...\n");
		if (!connected) {
			printf("a connection couldn't be established...\n");
			printf("Terminate program...\n");
			return -1;
		}
		//calling receive command procedure
		receive_commands();
		close(receiving_socket);
	}
	close(mysocket);
	freeLocoAPI();
	return 0;
}
