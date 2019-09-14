#include <iostream>
#include <stdio.h>
using namespace std;


#include <LocoKitInterface.h>



/////////////////please set your configuration here

//////////choose the active wheels
//////////choose 0 or 1
//////////1 ----> the wheel is active, 0 -----> the wheel is not active
#define W1 0 //wheel 1
#define W2 1 //wheel 2
#define W3 0 //wheel 3
#define W4 0 //wheel 4

/////////choose the direction
/////////1 -----> forward, 0 ----> backward
#define dW1 1 //wheel 1
#define dW2 1 //wheel 2
#define dW3 1 //wheel 3
#define dW4 1 //wheel 4


///////////choose the tourqe
#define T 300





/////////////////////////////////////////////////////////////////////////
//High level commands - can be designed according to the needed task/////
/////////////////////////////////////////////////////////////////////////
void stop_actuators(LocoKitInterface* LKI) {
	LKI->setActuatorStopped(2);
	LKI->setActuatorStopped(1);
	LKI->setActuatorStopped(6);
	LKI->setActuatorStopped(12);
}

void controller(LocoKitInterface* LKI) {

	stop_actuators(LKI);
	//wheel 1
	if (W1) {
		if (dW1)
			LKI->setActuatorPWM(T, 2);
		else
			LKI->setActuatorPWM(-T, 2);
	}

	//wheel 2
	if (W2) {
		if (dW2)
			LKI->setActuatorPWM(T, 12);
		else
			LKI->setActuatorPWM(-T, 12);
	}

	//wheel 3
	if (W3) {
		if (dW3)
			LKI->setActuatorPWM(T, 1);
		else
			LKI->setActuatorPWM(-T, 1);
	}

	//wheel 4
	if (W4) {
		if (dW4)
			LKI->setActuatorPWM(T, 6);
		else
			LKI->setActuatorPWM(-T, 6);
	}


	// read sensor

  LKI->updateSensorValueRawFloat_array();
    float f;
    for (int j=0;j <6; j++) {
      printf("%f  ", LKI->sensory_inputs[j]);
    }
    printf("\n");
    sleep(0.2);
    /////////

}

int main(int argc, char *argv[])
{
	LocoKitInterface LKI;
	if (LKI.establish_connection() == -1) {
		printf("Error from LocoKitInterface: a connection couldn't be established...\n");
		return -1;
	}
	printf("connected to robot...\n");
	controller(&LKI);
	printf("enter any key to quit and close the connection...\n");
	char x;
	cin>>x;
	stop_actuators(&LKI);
	LKI.terminate_connection_with_server();
	return 0;
}
