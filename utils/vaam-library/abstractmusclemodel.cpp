/*
 * abstractMuscleModel.cpp
 *
 *  Created on: 27.10.2014
 *      Author: Johannes Widenka
 */

#include "abstractmusclemodel.h"

SigmoidTransitionFunction defaultParameterTransition = DEFAULT_TRANSITION;
MuscleTorque1 defaultMuscleTorque = DEFAULT_MUSCLE_TORQUE;

AbstractMuscleModel::AbstractMuscleModel(MuscleModelConfiguration &c) {
	fK = c.Ktransf.clone();
	fD = c.Dtransf.clone();
	fM = c.torque.clone();
	this->unloadDFactor = c.Dmin;
	this->unloadKFactor = c.Kmin;
	this->K = c.K;
	this->D = c.D;
	this->timeSteps = c.timestep;
	this->length = c.length;
	this->radius = c.radius;
	this->mass = c.mass;
	this->zeroAngle = c.centerAngle;
	this->groundAngle = c.groundAngle;
	this->mActFactor = c.mActFactor;
	curD = 0;
	curK = 0;

}

AbstractMuscleModel::~AbstractMuscleModel() {
	delete fK;
	delete fD;
	delete fM;
}


double AbstractMuscleModel::getSignal(double muscleAct, double load,
		double angle,double aSpeed) {
	return getSignal(muscleAct,load,angle,aSpeed,0,0);

}

double AbstractMuscleModel::getSignal(double muscleAct, double load,
		double angle, double aSpeed, double exAngle, double disVec) {

	double exForce = load*mass;
	curK = K*(unloadKFactor+(1-unloadKFactor)*fK->getValue(load > 0.1?true:false,timeSteps));
	curD = D*(unloadDFactor+(1-unloadDFactor)*fD->getValue(load > 0.1?true:false,timeSteps));
	//printf("K: %f,D: %f, exforce:%f \n",curK, curD,exForce);

	return getOutput(muscleAct,exForce,angle-zeroAngle, getGroundAngle(angle),aSpeed,curK,curD,disVec);
}

double AbstractMuscleModel::getLocalDisVec(double angle) {
	return sin(getGroundAngle(angle))*length;
}

double AbstractMuscleModel::getGroundAngle(double angle){
	return angle-groundAngle;
}

double *AbstractMuscleModel::getCurKAddress() {
	return &curK;
}

double *AbstractMuscleModel::getCurDAddress() {
	return &curD;
}
