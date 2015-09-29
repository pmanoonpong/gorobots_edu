/*
 * sigmoidTransitionFunction.h
 *
 *  Created on: 03.11.2014
 *      Author: Johannes Widenka
 */

#ifndef SIGMOIDTRANSITIONFUNCTION_H_
#define SIGMOIDTRANSITIONFUNCTION_H_

#include "parametertransitionfunction.h"

/**
 * As the name tells, this ParameterTransitionFunction implementation defines a Sigmoid transition graph
 */
class SigmoidTransitionFunction: public ParameterTransitionFunction {
public:
	SigmoidTransitionFunction(int x):ParameterTransitionFunction(x){};
	SigmoidTransitionFunction* create(int length = 100) const;
	SigmoidTransitionFunction* clone() const;
private:
	virtual double f(double x);
	double K = 0.8; //sharpness of S-Curve
};

#endif /* SIGMOIDTRANSITIONFUNCTION_H_ */
