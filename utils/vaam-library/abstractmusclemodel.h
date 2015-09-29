/*
 * abstractMuscleModel.h
 *
 *  Created on: 27.10.2014
 *      Author: Johannes Widenka
 */

#ifndef ABSTRACTMUSCLEMODEL_H_
#define ABSTRACTMUSCLEMODEL_H_

#include "muscletorque1.h"
#include "sigmoidtransitionfunction.h"
#include <limits>
#include <stdio.h> //debug

#define DEFAULT_TRANSITION SigmoidTransitionFunction(50)
#define DEFAULT_MUSCLE_TORQUE MuscleTorque1()

extern SigmoidTransitionFunction defaultParameterTransition;
extern MuscleTorque1 defaultMuscleTorque;
//If other torquefunction or transition is wanted, set in constructor of MuscleModelConfiguration
/**
 * The structure holding the parameters for a muscle model. Changes to the configuration after creation of
 * the muscle mode, do not affect it in any way
 */
typedef struct MuscleModelConfiguration_t {
	ParameterTransitionFunction &Ktransf;
	ParameterTransitionFunction &Dtransf;
	MuscleTorqueFunction &torque;
	/// the duration between two calls to the muscle model in milliseconds
	double timestep;
	/// the maximum stiffness under full load
	double K;
	/// the maximum damping if totally unload
	double D;
	/// the minimal rate of stiffness (relative to K) if totally unload
	double Kmin;
	/// the minimal rate of damping (relative to D) if totally under load
	double Dmin;
	/// the mass, load on the joint, where the virtual muscles apply
	double mass;
	/// the length of the rotated element, controlled by the virtual muscles
	double length;
	/// the distance of the muscles to the rotation point
	double radius;
	/// the joint angle (radians), where the joint is in the center of both symmetric muscles
	double centerAngle;
	///the joint angle (radians), where the joint points directly to the ground (e.g.perfect vertical)
	double groundAngle;
	/// the muscle activation factor to scale the influence of the muscle signals
	double mActFactor;
	/**
	 *
	 * @param KparamTrans : the Transition Function that describes the change in K
	 * @param DparamTrans : the Transition Function that describes the change in D
	 * @param muscleTorqueF : the torque function being used by the muscles model
	 */
	MuscleModelConfiguration_t(ParameterTransitionFunction &KparamTrans =
			defaultParameterTransition,
			ParameterTransitionFunction &DparamTrans =
					defaultParameterTransition,
			MuscleTorqueFunction &muscleTorqueF = defaultMuscleTorque) :
			Ktransf(KparamTrans), Dtransf(DparamTrans), torque(muscleTorqueF) {
		timestep = 10;
		K = 1.0;
		D = 1.0;
		Kmin = .0;
		Dmin = 1;
		mass = 1.0;
		length = 1.0;
		radius = 0.01;
		centerAngle = .0;
		groundAngle = .0;
		mActFactor = 1.0;
	}
} MuscleModelConfiguration;
/* Functions for default configuration: */

extern MuscleModelConfiguration defaultMuscleConfig;

/**
 * The base class for muscle models, implementing most of it's functionality
 * subclasses essentially need to implement the method getOutput()
 */
class AbstractMuscleModel {
public:
	/**
	 * Standard constructor for muscle model
	 * @param conf: the configuration of the muscles mode. If not given, default parameters are used
	 */
	AbstractMuscleModel(MuscleModelConfiguration &conf = defaultMuscleConfig);
	virtual ~AbstractMuscleModel();

	/**
	 * Call this method to acquire the motor signal from the muscle model with given input values.
	 * This method uses no external angle, or displacement vector. It is meant for single joints, not depending on other
	 * muscle model affected joints.
	 * @param muscleAct: the activation of the muscle (usually the output of the basic controller)
	 * @param load: the current load rate on the joint. 0.0 means not touching the ground, 1.0 means that it fully supports
	 * the modeled mass, and is the only one supporting it. E.g. for biped standing with both feet it should be 0.5
	 * @param angle: the current angle (sensor) value, if it is not known by the model already (implementation dependent)
	 * @param aSpeed: the current rotational speed of the joint. May also be known by the model already (implementation dependent
	 * e.g. calculated by last and current angle)
	 * @return the muscle model modified motor command
	 */
	double getSignal(double muscleAct, double load, double angle = 0.0,
			double aSpeed = 0.0);

	/**
	 * Call this method to acquire the motor signal from the muscle model with given input values.
	 * This method uses the external angle as an offset for the angle for the local displacement vector calculation and
	 * an external displacement vector for cases that the joint is not directly connected to the ground at the end of it's length.
	 * @param muscleAct: the activation of the muscle (usually the output of the basic controller)
	 * @param load: the current load rate on the joint. 0.0 means not touching the ground, 1.0 means that it fully supports
	 * the modeled mass, and is the only one supporting it. E.g. for biped standing with both feet it should be 0.5
	 * @param angle: the current angle (sensor) value, if it is not known by the model already (implementation dependent)
	 * @param aSpeed: the current rotational speed of the joint. May also be known by the model already (implementation dependent
	 * e.g. calculated by last and current angle)
	 * @param exAngle: defines the orientation of the joint's source
	 * @param disVec: an external displacement Vector, used for torque calculation
	 * @return the muscle model modified motor command
	 */
	double getSignal(double muscleAct, double load, double angle, double aSpeed,
			double exAngle, double disVec);

	/**
	 * Method to retrieve the local displacement Vector, dependent on the joints current orientation and its length
	 * @param angle the current orientation of the joint
	 * @return
	 */
	double getLocalDisVec(double angle);

	double getGroundAngle(double angle);

	/*
	 * usual getters and setters..
	 *
	 */

	double getLength() const {
		return length;
	}

	void setLength(double length) {
		this->length = length;
	}

	double getMass() const {
		return mass;
	}

	void setMass(double mass) {
		this->mass = mass;
	}

	double getRadius() const {
		return radius;
	}

	void setRadius(double radius) {
		this->radius = radius;
	}

	double getTimeSteps() const {
		return timeSteps;
	}

	void setTimeSteps(double timeSteps) {
		this->timeSteps = timeSteps;
	}

	double *getCurKAddress(); //make these observable
	double *getCurDAddress(); // ´´

	double K; 				//Stiffness coefficient under full load
	double D; 				//Damping coefficient under full load
	double unloadKFactor; //K under total unload (Swing phase)= K*unloadKFactor
	double unloadDFactor; //D under total unload (Swing phase)= D*unloadKFactor
protected:
	/**
	 * This function has to be implemented by subclasses (non-abstract muscle model implementations).
	 * Usually, it will call the muscle models torque function at some point.
	 * This function is called by getSignal(), which directly propagates this functions return value.
	 * @param muscleAct: the activation of the muscle (usually the output of the basic controller)
	 * @param exForce: the current load on the muscle (already including the simulated mass)
	 * @param angleToCenter: derivation of the current angle to the center angle (in terms of the symmetric muscles) in radians.
	 * @param angleToGround: derivation of the current angle to the angle, where it directly points towards the ground (or where ever the load is applied as a force) in radians.
	 * @param aSpeed: the current speed of the joint
	 * @param K: current stiffness parameter for the muscles model
	 * @param D: current damping parameter for the muscles model
	 * @param DisVec: the horizontal displacement of force application
	 * @return the motor signal values
	 */
	virtual double getOutput(double muscleAct, double exForce,
			double angleToCenter, double angleToGround, double aSpeed, double K,
			double D, double DisVec) = 0;
	double zeroAngle; //angle input where the virtual muscles are as relaxed as possible  (center position without load or activation of muscles)
	double groundAngle; //angle input where the joint points to gravity direction (full support of body)
	int timeSteps;	//duration of one timestep in miliseconds
	double mass;	//mass loaded on the muscles
	double length;	//length of the muscles
	double radius;	//radius of the joint
	double mActFactor; //muscleActivation multiplier
	double curK;	//current K and D (calculated before torque calculation)
	double curD;	//
protected:
	ParameterTransitionFunction *fK;	//pointer to the function that is used to describe the change of K during swing/stance phase
	ParameterTransitionFunction *fD;	//pointer to the function that is used to describe the change of D during swing/stance phase
	MuscleTorqueFunction *fM;
};

#endif /* ABSTRACTMUSCLEMODEL_H_ */
