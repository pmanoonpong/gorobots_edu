/*
 * muscleChain.cpp

 *
 *  Created on: 23.01.2015
 *      Author: Johannes Widenka
 */

#include "musclechain.h"

//the lowest index of muscle is the most distant to the ground. the highest index is the closest to the ground

MuscleChain::MuscleChain(int size) {
	chainSize = size;
	isValid = false;
	params = new MuscleInput[size];
	muscles = new AbstractMuscleModel*[size];
	for (int i = 0; i < size; i++)
		muscles[i] = 0;
}

MuscleChain::~MuscleChain() {
	delete[] muscles;
	delete[] params;
}

void MuscleChain::addMuscle(int index, AbstractMuscleModel* muscle) {
	if ((index > -1) && (index < chainSize)) {
		muscles[index] = muscle;
		params[index] = MuscleInput();
		isValid = false;

	}

}

void MuscleChain::removeMuscle(int index) {
	if ((index > -1) && (index < chainSize)) {
		muscles[index] = 0;
		isValid = false;
	}
}

void MuscleChain::setState(int index, double motorCommand, double load,
		double jointAngle, double rotationalSpeed) {
	if ((index > -1) && (index < chainSize) && (muscles[index] != 0)) {
		params[index].angle = jointAngle;
		params[index].load = load;
		params[index].speed = rotationalSpeed;
		params[index].motorCommand = motorCommand;
		isValid = false;
	}
}

double MuscleChain::getSignal(int index) {
	if ((index > -1) && (index < chainSize) && (muscles[index] != 0)) {
		if (!isValid)
			validateModel();
		return muscles[index]->getSignal(params[index].motorCommand,
				params[index].load, params[index].angle, params[index].speed,
				params[index].exAngle, params[index].disVec);
	}
	return 0;
}

AbstractMuscleModel* MuscleChain::getMuscle(int index) {
	if ((index > -1) && (index < chainSize))
		return muscles[index];
	return 0;
}

void MuscleChain::validateModel() {
	double tempAngle = 0;
	double tempdVec = 0;
	for (int i = 0; i < chainSize; i++) {
		if (muscles[i] != 0) {
			params[i].exAngle = tempAngle;
			tempAngle += muscles[i]->getGroundAngle(params[i].angle);
		}

	}

	for (int i = chainSize - 1; i > -1; i--) {
		if (muscles[i] != 0) {
			params[i].disVec = tempdVec;
			tempdVec += muscles[i]->getLocalDisVec(
					params[i].angle + params[i].exAngle);
		}
	}
	isValid = true;
}
