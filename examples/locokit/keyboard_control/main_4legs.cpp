#include <iostream>
#include <stdio.h>
using namespace std;


#include <LocoKitInterface.h>



/////////////////////////////////////////////////////////////////////////
//High level commands - can be designed according to the needed task/////
/////////////////////////////////////////////////////////////////////////
void stop_actuators(LocoKitInterface* LKI) {
	LKI->setActuatorStopped(2);
	LKI->setActuatorStopped(1);
	LKI->setActuatorStopped(3);
	LKI->setActuatorStopped(34);
}

void go_forward_pwm(LocoKitInterface* LKI, float pmw) {
	stop_actuators(LKI);
	LKI->setActuatorPWM(pmw, 2);
	LKI->setActuatorPWM(pmw, 3);
	LKI->setActuatorPWM(pmw, 1);
	LKI->setActuatorPWM(pmw, 34);
}

////////////another version of forward moving using setConstantSpeedInterpolatingFunction////////////
void go_forward_ConstantSpeedInterpolating(LocoKitInterface* LKI, float period, float phaseoffset) {
	stop_actuators(LKI);
	LKI->setConstantSpeedInterpolatingFunction(2, period, phaseoffset, 0);
	LKI->setConstantSpeedInterpolatingFunction(12, period, phaseoffset, 0);
	LKI->setConstantSpeedInterpolatingFunction(6, period, phaseoffset, 1);
	LKI->setConstantSpeedInterpolatingFunction(1, period, phaseoffset, 1);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////


void go_backward_pwm(LocoKitInterface* LKI, float pmw) {
	stop_actuators(LKI);
	LKI->setActuatorPWM(-pmw, 2);
	LKI->setActuatorPWM(-pmw, 12);
	LKI->setActuatorPWM(pmw, 1);
	LKI->setActuatorPWM(pmw, 6);
}

void go_right_pwm(LocoKitInterface* LKI, float pmw) {
	stop_actuators(LKI);
	LKI->setActuatorPWM(pmw, 2);
	LKI->setActuatorPWM(pmw, 12);
	LKI->setActuatorPWM(-pmw/4.0, 1);
	LKI->setActuatorPWM(-pmw/4.0, 6);
}

void go_left_pwm(LocoKitInterface* LKI, float pmw) {
	stop_actuators(LKI);
	LKI->setActuatorPWM(pmw/4.0, 2);
	LKI->setActuatorPWM(pmw/4.0, 12);
	LKI->setActuatorPWM(-pmw, 1);
	LKI->setActuatorPWM(-pmw, 6);
}
/////////////////////////////////////////////////////////////////////////
void get_current_sensors_values_set1(LocoKitInterface* LKI) {
	//accx  accy  accz  gyox  gyoy  gyoz
	system("stty cooked");
	printf("\n");
	printf("M1_pmw M2_pmw M3_pmw M4_pmw \t M1_velocity M2_velocity M3_velocity M4_velocity\n");
	for (int i=0; i<20; i++) {
		float f;
		LKI->getActuatorPWM(1, 	f);
		printf("%f  ", f);
		LKI->getActuatorPWM(2, f);
		printf("%f  ", f);
		LKI->getActuatorPWM(6, f);
		printf("%f  ", f);
		LKI->getActuatorPWM(12, f);
		printf("%f  ", f);
		printf("\t");

		LKI->getActuatorVelocity(1, f);
		printf("%f  ", f);
		LKI->getActuatorVelocity(2, f);
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
		LKI->getActuatorPosition(1, 	f);
		printf("%f  ", f);
		LKI->getActuatorPosition(2, f);
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



void controller(LocoKitInterface* LKI) {
	// Set terminal to raw mode
	system("stty raw");
	float main_pmw = 400;
	// Wait for single character
	while (1) {
		char input = getchar();
		if (input == 'w') {
			go_forward_pwm(LKI, main_pmw);
			//go_forward_ConstantSpeedInterpolating(LKI, 1, 90);
		} else if (input == 's') {
			go_backward_pwm(LKI, main_pmw);
		} else if (input == 'd') {
			go_right_pwm(LKI, main_pmw);
		} else if (input == 'a') {
			go_left_pwm(LKI, main_pmw);
		} else if (input == 'p') {
			stop_actuators(LKI);
		} else if (input == 'i') {
			//get_current_sensors_values_set1(LKI);
			get_current_sensors_values_set2(LKI);
			//get_current_sensors_values_set3(LKI);
		} else if (input == 'q') {
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

	// get velocity of module 6
	float f;
	LKI.getActuatorVelocity(6, f);
  printf("velocity... %f \n", f);
	//
	printf("connected to robot...\n");
	controller(&LKI);
	return 0;
}
