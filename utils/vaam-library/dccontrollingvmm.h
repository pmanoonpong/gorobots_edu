/*
 * DCControlingVMM.h
 *
 *  Created on: 06.11.2014
 *      Author: Johannes Widenka
 */

#ifndef DCCONTROLINGVMM_H_
#define DCCONTROLINGVMM_H_

#include "abstractmusclemodel.h"

/**
 * This class is meant to apply the muscle model to control values, designed to control a DC motor. It assumes a negative or positive
 * value, where the sign defines the direction and the absolut value the speed and the strength of the movement. It outputs
 * similar values, just modified the by muscle models properties (spring/ damper and simulated external force). It is not guaranteed
 * to have output values of the same scale, so this may have to be scaled to the motors expectation after call on getSignal( .. )
 */
class DCControllingVMM: public AbstractMuscleModel {
public:
	/**
	 * Standard constructor for the DC controlling VMM (virtual muscle model). It generates a muscle model, with the
	 * pre angle value, needed for speed calculation set to zero.
	 * @param config the configuration of the muscles mode. If not given, default parameters are used
	 */
	DCControllingVMM(MuscleModelConfiguration &config):AbstractMuscleModel(config){preAngle = 0;}
protected:
	/**
	 * This function calculates a target torque out of the current motor and muscle parameters, which may be used to controll
	 * the voltage of a DC motor. It returns negative values for the opposite direction. It is called by the getSignal(..) implementations, which is the the correct
	 * function to access the models outcome.
	 * @param muscleAct: the activation of the muscle (usually the output of the basic controller)
	 * @param exForce: the current load on the muscle (already including the simulated mass)
	 * @param angleToCenter: derivation of the current angle to the center angle (in terms of the symmetric muscles) in radians.
	 * @param angleToGround: derivation of the current angle to the angle, where it directly points towards the ground (or where ever the load is applied as a force) in radians.
	 * @param aSpeed: the current speed of the joint
	 * @param K: current stiffness parameter for the muscles model
	 * @param D: current damping parameter for the muscles model
	 * @param DisVec: the horizontal displacement of force application
	 * @return the motor signal value. It should be proportional to the DC-Power and may have to be scaled
	 */
	double getOutput(double muscleAct, double exForce, double angleToCenter, double angleToGround, double aSpeed, double K, double D, double DisVec );
private:
	double preAngle;
};

#endif /* DCCONTROLINGVMM_H_ */
