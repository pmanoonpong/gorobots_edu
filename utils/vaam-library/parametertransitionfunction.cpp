/*
 * parameterTransitionFunction.cpp
 *
 *  Created on: 29.10.2014
 *      Author: Johannes Widenka
 */

#include "parametertransitionfunction.h"

ParameterTransitionFunction::ParameterTransitionFunction(int length) {
	curStep = 0;
	timeMult = length;
}

ParameterTransitionFunction::~ParameterTransitionFunction() {
}

double ParameterTransitionFunction::getValue(bool positiveDirection,
		int time_millis) {
	curStep += positiveDirection?time_millis:-time_millis;
	if (curStep > timeMult) curStep = timeMult;
	if (curStep < 0) curStep = 0;
	double r = f(curStep/timeMult);
	//printf("step: %f, ret: %f",curStep/timeMult,r);
	if (r < 0.0) return 0.0;
	if (r > 1.0) return 1.0;
	return r;
}
