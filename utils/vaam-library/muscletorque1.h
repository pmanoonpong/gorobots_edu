/*
 * muscleTorque1.h
 *
 *  Created on: 05.11.2014
 *      Author: Johannes Widenka
 */

#ifndef MUSCLETORQUE1_H_
#define MUSCLETORQUE1_H_

/*
 * TODO: remove muscle torque class and integrate into abstractMuscleModel.
 * It became somehow useless to use an own class for the implementation, since
 * the torque calculation is rather general, and is not expected to be replaced.
 */

#include "muscletorquefunction.h"
#include <math.h>

/**
 * A symmentric agonist antagonist muscle model torque implementation based on the model of Xiaofeng Xiong
 */
class MuscleTorque1: public MuscleTorqueFunction {
public:
	MuscleTorque1 *create() const;
	MuscleTorque1 *clone() const;
	/**
	 * Torque calculation composed out of external force (weight or load on the joint),
	 * passive force (spring/damper system, modeling physical muscle parameters)
	 * and active force, the muscles activation.
	 * @param AngToCenter angular offset (radian) to the maximum relaxed position of the symmetric muscles
	 * @param AngToGround angular offset (radian) to the vertical pointing vector
	 * @param DAng rotation speed of the joint
	 * @param Exforce the external force, pushing the joint to the ground
	 * @param MuscleAct muscles activity
	 * @param DisVe	the external displacement vector, being added to the local displacement before calculation of external force
	 * @param K stiffness factor
	 * @param D damping factor
	 * @param Leng length of the joint
	 * @param Radius radius of the joint, defining the distance where the muscles apply their force (passive and active force) to the joint
	 * @param MusclActFac muscle activation factor
	 * @return the torque based on raw input into this function
	 */
	double getTorque(double AngToCenter, double AngToGround, double DAng, double Exforce, double MuscleAct, double DisVe, double K, double D, double Leng, double Radius, double MusclActFac);
};

#endif /* MUSCLETORQUE1_H_ */
