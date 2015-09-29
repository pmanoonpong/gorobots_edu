/*
 * DCControlingVMM.cpp
 *
 *  Created on: 06.11.2014
 *      Author: Johannes Widenka
 */

#include "dccontrollingvmm.h"

double DCControllingVMM::getOutput(double muscleAct, double exForce,
		double angleToCenter, double angleToGround, double aSpeed, double K, double D, double DisVec) {
	//double ret = fM->getTorque(angleToCenter,angleToGround,(angleToCenter-preAngle)/(timeSteps/1000.0),exForce,muscleAct,DisVec,K,D,length,radius,mActFactor)/(mass * radius * radius / 2.0);
	double ret = fM->getTorque(angleToCenter,angleToGround,(angleToCenter-preAngle)/(timeSteps/1000.0),exForce,muscleAct,DisVec,K,D,length,radius,mActFactor)/*/(mass * radius * radius / 2.0)*/;
	preAngle = angleToCenter;
	return ret;
}
