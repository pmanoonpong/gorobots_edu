/*
 * parameterTransitionFunction.h
 *
 *  Created on: 29.10.2014
 *      Author: Johannes Widenka
 */

#ifndef PARAMETERTRANSITIONFUNCTION_H_
#define PARAMETERTRANSITIONFUNCTION_H_

#include <stdio.h>
/**
 * Abstract base class for parameter transition functions. Usually it's implementations should describe a
 * function that defines a graph from [0,<0] to [1,>1], which will cut to [0,1] within this class.
 * In VAAM it is used to describe a smooth transition of the stiffness and damping value when the joint changes
 * from stance to swing phase and the other way around.
 */
class ParameterTransitionFunction {
	public:
		/**
		 * pattern function, because you can not give a class name as parameter
		 * @param length
		 * @return
		 */
		virtual ParameterTransitionFunction *create(int length = 100) const = 0;
		/**
		 * pattern function, because you can not give a class name as parameter
		 * @param length
		 * @return
		 */
		virtual ParameterTransitionFunction *clone() const = 0;
		/**
		 * Standard constructor for a ParameterTransitionFunction
		 * @param length time in milliseconds, the function needs for a full transition
		 */
		ParameterTransitionFunction(int length = 100); //time in milliseconds, the function needs for a full transition
		virtual ~ParameterTransitionFunction();

		/**
		 * retrieve the current transition value after a certain time passed, moving to 0 direction or to 1. The
		 * transition is symmetric.
		 * @param positiveDirection true if the transition should move from 0.0 to 1.0. false otherwise.
		 * @param time_millis how much time passed since last call (how many time steps should the transition move)
		 * @return the current transition value, [0.0  ..  1.0]
		 */
		double getValue(bool positiveDirection, int time_millis); //positive direction should be true if transfer from 0 to 1
	protected:
		/**
		 * the function should map from 0.0 .. 1.0 in 0.0 .. 1.0
		 * This should be implemented by the subclasses to define the shape of the transition.
		 * @param x
		 * @return
		 */
		virtual double f(double x)=0;
	private:
		double timeMult;
		int curStep;	//currentStep
};

#endif /* PARAMETERTRANSITIONFUNCTION_H_ */
