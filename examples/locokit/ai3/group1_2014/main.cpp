#include <iostream>
#include <stdio.h>
using namespace std;


#include <LocoKitInterface.h>



/////////////////////////////////////////////////////////////////////////
//High level commands - can be designed according to the needed task/////
/////////////////////////////////////////////////////////////////////////
void stop_actuators(LocoKitInterface* LKI) {
	LKI->setActuatorStopped(4);
	LKI->setActuatorStopped(31);
	LKI->setActuatorStopped(6);
	LKI->setActuatorStopped(12);
}

void go_forward_WalkGait(LocoKitInterface* LKI, float period, float phaseoffset) {
	stop_actuators(LKI);
	
	//Front legs, opposite direction
	//Right
	LKI->setConstantSpeedInterpolatingFunction(31, 0.75, 90, 1); 
	//Left
	LKI->setConstantSpeedInterpolatingFunction(6, 0.75, 270, 0); 

	//Back legs, opposite direction
	//Right
	LKI->setConstantSpeedInterpolatingFunction(4, 0.75, 180, 1); 
	//Left
	LKI->setConstantSpeedInterpolatingFunction(12, 0.75, 0, 0); 
}

void go_forward_BoundGait(LocoKitInterface* LKI, float period, float phaseoffset) {
	stop_actuators(LKI);

	//Front legs, opposite direction
	//Right
	LKI->setConstantSpeedInterpolatingFunction(31, 0.35, 5, 0);
	//Left
	LKI->setConstantSpeedInterpolatingFunction(6, 0.35, 185, 1);

	//Back legs, opposite direction
	//Right
	LKI->setConstantSpeedInterpolatingFunction(4, 0.35, 180, 0);
	//Left
	LKI->setConstantSpeedInterpolatingFunction(12, 0.35, 0, 1);
}

void go_forward_TrotGait(LocoKitInterface* LKI, float period, float phaseoffset) {
	stop_actuators(LKI);
	
	//Front legs, opposite direction
	//Right
	LKI->setConstantSpeedInterpolatingFunction(31, 0.75, 180, 1);
	//Left
	LKI->setConstantSpeedInterpolatingFunction(6, 0.75, 180, 0);

	//Back legs, opposite direction
	//Right
	LKI->setConstantSpeedInterpolatingFunction(4, 0.75, 0, 1);
	//Left
	LKI->setConstantSpeedInterpolatingFunction(12, 0.75, 0, 0);
}

void go_forward_PaceGait(LocoKitInterface* LKI, float period, float phaseoffset) {
	stop_actuators(LKI);
	
	//Front legs, opposite direction
	//Right
	LKI->setConstantSpeedInterpolatingFunction(31, 0.75, 90, 0);
	//Left
	LKI->setConstantSpeedInterpolatingFunction(6, 0.75, 90, 1);

	//Back legs, opposite direction
	//Right
	LKI->setConstantSpeedInterpolatingFunction(4, 0.75, 270, 0);
	//Left
	LKI->setConstantSpeedInterpolatingFunction(12, 0.75, 270, 1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

void go_backward_ConstantSpeedInterpolating(LocoKitInterface* LKI, float period, float phaseoffset) {
	stop_actuators(LKI);
	
	//Front legs, opposite direction
	//Right
	LKI->setConstantSpeedInterpolatingFunction(31, period, 0, 1);
	//Left
	LKI->setConstantSpeedInterpolatingFunction(6, period, 180, 0);

	//Back legs, opposite direction
	//Right
	LKI->setConstantSpeedInterpolatingFunction(4, period, 90, 1);
	//Left
	LKI->setConstantSpeedInterpolatingFunction(12, period, 90, 0);
}

/////////////////////////////////////////////////////////////////////////

void get_current_sensors_values_set1(LocoKitInterface* LKI) {
	//accx  accy  accz  gyox  gyoy  gyoz
	system("stty cooked");
	printf("\n");
	printf("M1_pmw M2_pmw M3_pmw M4_pmw \t M1_velocity M2_velocity M3_velocity M4_velocity\n");
	for (int i=0; i<20; i++) {
		float f;
		LKI->getActuatorPWM(4, 	f);
		printf("%f  ", f);
		LKI->getActuatorPWM(31, f);
		printf("%f  ", f);
		LKI->getActuatorPWM(6, f);
		printf("%f  ", f);
		LKI->getActuatorPWM(12, f);
		printf("%f  ", f);
		printf("\t");

		LKI->getActuatorVelocity(4, f);
		printf("%f  ", f);
		LKI->getActuatorVelocity(31, f);
		printf("%f  ", f);
		LKI->getActuatorVelocity(6, f);
		printf("%f  ", f);
		LKI->getActuatorVelocity(12, f);
		printf("%f  ", f);

		printf("\n");
		sleep(0.5);
	}
	system("stty raw");
}

void get_current_sensors_values_set2(LocoKitInterface* LKI) {
	//accx  accy  accz  gyox  gyoy  gyoz
	system("stty cooked");
	printf("\n");
	printf("M1_pos M2_pos M3_pos M4_pos\n");
	for (int i=0; i<20; i++) {
		float f;
		LKI->getActuatorPosition(4, 	f);
		printf("%f  ", f);
		LKI->getActuatorPosition(31, f);
		printf("%f  ", f);
		LKI->getActuatorPosition(6, f);
		printf("%f  ", f);
		LKI->getActuatorPosition(12, f);
		printf("%f  ", f);
		printf("\t");
		printf("\n");
		sleep(0.2);
	}
	system("stty raw");
}

void get_current_sensors_values_set3(LocoKitInterface* LKI) {
	system("stty cooked");
	printf("\n");
	printf("accx  accy  accz  gyox  gyoy  gyoz\n");
	for (int i=0; i<20; i++) {
		LKI->updateSensorValueRawFloat_array();
		float f;
		for (int j=0;j <6; j++) {
			printf("%f  ", LKI->sensory_inputs[j]);
		}
		printf("\n");
		sleep(0.2);
	}
	system("stty raw");
}

void get_current_position_values(LocoKitInterface* LKI) {

	system("stty cooked");

	float positions[4]={0};

	LKI->getActuatorPosition(6,positions[0]);	
	LKI->getActuatorPosition(31,positions[1]);
	LKI->getActuatorPosition(12,positions[2]);
	LKI->getActuatorPosition(4,positions[3]);

	printf("Actuator Positions\n");
	printf("FL FR BL BR\n");

	for (int i=0; i<4; i++)
		printf("%d  ",(int)positions[i]%360);

	printf("\n");

	system("stty raw");
}	

void get_current_velocity_values(LocoKitInterface* LKI) {

	system("stty cooked");

	float velocities[4]={0};

	LKI->getActuatorVelocity(6,velocities[0]);	
	LKI->getActuatorVelocity(31,velocities[1]);
	LKI->getActuatorVelocity(12,velocities[2]);
	LKI->getActuatorVelocity(4,velocities[3]);

	printf("Actuator Velocities\n");
	printf("FR  FR  BL  BR\n");

	for (int i=0; i<4; i++)
		printf("%f  ",velocities[i]);
	
	printf("\n");

	system("stty raw");
}	

void controller(LocoKitInterface* LKI) {
	// Set terminal to raw mode
	system("stty raw");
	float main_pmw = 450;
	// Wait for single character
	while (1) {
		char input = getchar();
		if (input == '1') {
			go_forward_WalkGait(LKI, 1, 90);
		} else if (input == '2') {
			go_forward_TrotGait(LKI, 1, 90);
		} else if (input == '4') {
			go_forward_BoundGait(LKI, 1, 90);
		} else if (input == '3') {	
			go_forward_PaceGait(LKI, 1, 90);
		} else if (input == 's') {
			//go_backward_pwm(LKI, main_pmw);
			go_backward_ConstantSpeedInterpolating(LKI, 0.5, 90);
		} else if (input == 'p') {
			stop_actuators(LKI);
		} else if (input == 'i') {
			//get_current_sensors_values_set1(LKI);
			//get_current_sensors_values_set2(LKI);
			get_current_sensors_values_set3(LKI);
		} else if (input == 'x') {
			get_current_position_values(LKI);
		} else if (input == 'v') {
			get_current_velocity_values(LKI);	
		} else if (input == 'q') {
			stop_actuators(LKI);
			system("stty cooked");
			break;
		} else if (input == 't') {
			LKI->terminate_connection_with_server();
		}
	}
}

//LKI->setConstantSpeedInterpolatingFunction(2, 1, 90, 1);



int main(int argc, char *argv[])
{
	LocoKitInterface LKI;
	if (LKI.establish_connection() == -1) {
		printf("Error from LocoKitInterface: a connection couldn't be established...\n");
		return -1;
	}
	printf("connected to robot...\n");
	controller(&LKI);
	return 0;
}
