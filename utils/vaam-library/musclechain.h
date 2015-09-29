/*
 * muscleChain.h
 *
 *  Created on: 23.01.2015
 *      Author: Johannes Widenka
 */

#ifndef MUSCLECHAIN_H_
#define MUSCLECHAIN_H_

#include "abstractmusclemodel.h"


/**
 * This class handles the propagation of displacement vectors and external angles among a chain of connected joints.
 * The joint with the lowest index is the most distant to the ground, e.g. the hip (more precise: the point of force application). The joint with
 * the highest index refers to the joint closest to the ground, e.g. the foot. One may use arbitrary large chains, every index without a virtual
 * muscle connected to it is ignored.
 */
class MuscleChain {
public:
	/**
	 * Constructor for a muscle chain of known size. You may create a chain that is larger than the amount of joints that will finally be included
	 * empty indices are ignored.
	 * @param size: the size of the muscle chain
	 */
	MuscleChain(int size);
	~MuscleChain();

	/**
	 * Includes the given muscle model into the muscle chain at the given index
	 * @param index the position of the muscle in the muscle chain
	 * @param muscle the actual muscle model
	 */
	void addMuscle(int index, AbstractMuscleModel *muscle);

	/**
	 * deletes the muscle model from the list of chained muscles
	 * @param index the index of the muscle model that should be removed
	 */
	void removeMuscle(int index);

	/**
	 * Retrieve a pointer to the muscle model of a certain index
	 * @param index the index of the retrieved muscle model
	 * @return The pointer to the muscle mode or 0 if there is no model or it is out of bounds
	 */
	AbstractMuscleModel *getMuscle(int index);

	/**
	 * Set all input values, that are used when getSignal(int index) is being called the next time for this index
	 * @param index index of the muscle model
	 * @param motorCommand the motor command, or muscle activation for the model
	 * @param load the load (rate) in the range of [0 .. 1.0]
	 * @param jointAngle joint angle, if not retrieved by the model itself (e.g. servo motor control)
	 * @param rotationalSpeed the speed of the joint
	 */
	void setState(int index, double motorCommand, double load, double jointAngle = 0.0, double rotationalSpeed = 0.0);

	/**
	 * Returns the models output based on the input values of the last setState() call and the state of the connected joints.
	 * @param index identifies the muscle model which returns its output
	 * @return the motor muscle model modified motor signal (or more general: output value)
	 */
	double getSignal(int index);
protected:
	AbstractMuscleModel **muscles;	//pointers to the muscle models
	int chainSize;	//size of the muscle chain
	/**
	 * this structure is only used to hold the current input parameters for a muscle model
	 */
	typedef struct MuscleInput_t{
		double motorCommand = 0.0;
		double load = 0.0;
		double angle = 0.0;
		double speed = 0.0;
		double exAngle = 0.0;
		double disVec = 0.0;

	}MuscleInput;

	MuscleInput *params;	//array of input values, current state of them muscle chain
	bool isValid;	//model is invalid as soon as something changed, and needs to be validated to return a signal of any
					//muscle model

	/**
	 * the validation of the muscle model. Actual propagation and calculation of correct displacement vectors
	 */
	void validateModel();
};

#endif /* MUSCLECHAIN_H_ */
