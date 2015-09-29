/*
 * LocoKitController.cpp
 *
 *  Created on: Dec 27, 2014
 *      Author: leon
 */

#include <assert.h>

#include "SpringyBotEmptyController.h"
using namespace std;
using namespace matrix;

// The constructor implements the AbstractController interface. A trial number can be
// passed to the constuctor in order to automate variations in the controller setup
// over different trials.
SpringyBotEmptyController::SpringyBotEmptyController(const std::string& name)
: AbstractController(name, "1.0") {
	initialised=false;

	ticks_since_init = 0;
  
	speedSetpoint = 4.0;
	phaseSetpoint = 4.0;
	
}

SpringyBotEmptyController::~SpringyBotEmptyController() {
}

void SpringyBotEmptyController::stepNoLearning(const sensor* sensors, int number_sensors, motor* motors, int number_motors) {
}

void SpringyBotEmptyController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {

	// Update internal time
	ticks_since_init++;

	// Read sensors
	double leftFrontPosition = sensors[SIdx("left front motor")];
	double leftRearPosition = sensors[SIdx("left rear motor")];
	double rightFrontPosition = sensors[SIdx("right front motor")];
	double rightRearPosition = sensors[SIdx("right rear motor")];

	// Set motor speeds
	motors[MIdx("left front motor")] = speedSetpoint;
	motors[MIdx("right front motor")] = speedSetpoint;
	motors[MIdx("left rear motor")] = speedSetpoint;
	motors[MIdx("right rear motor")] = speedSetpoint;

}

void SpringyBotEmptyController::init(int sensornumber, int motornumber, RandGen* randGen) {
	nSensors = sensornumber;
	nMotors  = motornumber;
	initialised=true;
}

int SpringyBotEmptyController::getSensorNumber() const {
	return nSensors;
}

int SpringyBotEmptyController::getMotorNumber() const {
	return nMotors;
}

bool SpringyBotEmptyController::store(FILE* f) const {
	Configurable::print(f,0);
	return true;
}

bool SpringyBotEmptyController::restore(FILE* f) {
	Configurable::parse(f);
	return true;
}


