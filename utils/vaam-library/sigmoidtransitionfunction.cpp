/*
 * sigmoidTransitionFunction.cpp
 *
 *  Created on: 03.11.2014
 *      Author: Johannes Widenka
 */

#include "sigmoidtransitionfunction.h"


double g(double x,double k){
	return (k*x-x)/(2*k*x-k-1);
}

double SigmoidTransitionFunction::f(double x){
	if (x < 0.5)
		return (g(2*x,K)*0.5);
	else
		return (0.5*g(2*(x-0.5),-K)+0.5);
}

SigmoidTransitionFunction* SigmoidTransitionFunction::create(int length) const {
	return new SigmoidTransitionFunction(length);
}

SigmoidTransitionFunction* SigmoidTransitionFunction::clone() const {
		return new SigmoidTransitionFunction(*this);
}

