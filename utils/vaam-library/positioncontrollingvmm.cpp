/*
 * PositionControllingVMM.cpp
 *
 *  Created on: 13.11.2014
 *      Author: Johannes Widenka
 */

#include "positioncontrollingvmm.h"

double PositionControllingVMM::getOutput(double muscleAct, double exForce,
		double angleToCenter, double angleToGround, double aSpeed, double K, double D, double DisVec) {
	double x1,v1,a1,x2,v2,a2,x3,v3,a3,x4,v4,a4,xf,vf;

	x1 = preAngle;
	v1 = preSpeed;
	a1 = fM->getTorque(x1,x1+zeroAngle-groundAngle, v1, timeSteps, preExforce, preMusclAct, preDisVec, K, D, length, mActFactor)/(mass * radius * radius / 2.0);

	x2 = preAngle + (0.5 * v1 * timeSteps);
	v2 = preSpeed + (0.5 * a1 * timeSteps);
	a2 = fM->getTorque(x2,x2+zeroAngle-groundAngle, v2, timeSteps/2.0, (preExforce + exForce)/2.0, (preMusclAct + muscleAct)/2.0, (preDisVec + DisVec)/2.0, K, D, length, mActFactor)/(mass * radius * radius / 2.0);

	x3 = preAngle + (0.5 * v2 * timeSteps);
	v3 = preSpeed + (0.5 * a2 * timeSteps);
	a3 = fM->getTorque(x3,x3+zeroAngle-groundAngle, v3, timeSteps/2.0, (preExforce + exForce)/2.0, (preMusclAct + muscleAct)/2.0, (preDisVec + DisVec)/2.0, K, D, length, mActFactor)/(mass * radius * radius / 2.0);

	x4 = preAngle + (v3 * timeSteps);
	v4 = preSpeed + (a3 * timeSteps);
	a4 = fM->getTorque(x4,x4+zeroAngle-groundAngle, v4, timeSteps, exForce, muscleAct, DisVec, K, D, length, mActFactor)/(mass * radius * radius / 2.0);

	xf = preAngle + (timeSteps/6.0) * (v1 + (2 * v2) + (2 * v3) + v4);
	vf = preSpeed + (timeSteps/6.0) * (a1 + (2 * a2) + (2 * a3) + a4);

	//MusclePara MusPara1 = {xf, vf}; //is it needed to return the speed?

			preDisVec = DisVec;
	preAngle = xf;
	preSpeed = vf;
	preExforce = exForce;
	preMusclAct = muscleAct;

	return xf;
}
