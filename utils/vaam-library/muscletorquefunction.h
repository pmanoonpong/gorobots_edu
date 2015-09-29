/*
 * muscleTorqueFunction.h
 *
 *  Created on: 03.11.2014
 *      Author: Johannes Widenka
 */

#ifndef MUSCLETORQUEFUNCTION_H_
#define MUSCLETORQUEFUNCTION_H_

//TODO: remove muscle torque class and integrate into abstractMuscleModel. It became somehow useless

/**
 * This abstract class serves as an interface for torque implementations.
 */
class MuscleTorqueFunction {
public:
	virtual ~MuscleTorqueFunction(){

	};
	virtual MuscleTorqueFunction* create() const = 0;
	virtual MuscleTorqueFunction* clone() const = 0;
	virtual double getTorque(double AngToCenter, double AngToGround, double DAng, double Exforce, double MuscleAct, double DisVe, double K, double D, double Leng, double Radius, double MusclActFac) = 0;
};

#endif /* MUSCLETORQUEFUNCTION_H_ */
