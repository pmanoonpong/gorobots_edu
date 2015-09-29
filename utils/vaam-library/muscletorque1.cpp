/*
 * muscleTorque1.cpp
 *
 *  Created on: 05.11.2014
 *      Author: Johannes Widenka
 */

#include "muscletorque1.h"

MuscleTorque1* MuscleTorque1::create() const {
	return new MuscleTorque1();
}

MuscleTorque1* MuscleTorque1::clone() const {
	return new MuscleTorque1(*this);
}

double MuscleTorque1::getTorque(double AngToCenter, double AngToGround, double DAng, double Exforce,
		double MuscleAct, double DisVe, double K, double D, double Leng, double Radius,
		double MusclActFac) {

    double ExVec,ExforceTor,PasiForceTor,ActiForceTor;

    ExVec = (Leng + Radius);

    ExforceTor = Exforce * ((sin(AngToGround)*ExVec)+DisVe);
    //ExforceTor=0;
    PasiForceTor = (2 * ((D * DAng) + (K * AngToCenter)) * Radius) * Radius;
    ActiForceTor = MuscleAct * Radius *MusclActFac;

    return ExforceTor + ActiForceTor - PasiForceTor;
}
