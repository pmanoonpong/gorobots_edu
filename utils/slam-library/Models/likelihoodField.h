#ifndef SB_LIKELIHOOD_H_
#define SB_LIKELIHOOD_H_

#include "measurementModelAbstract.h"
#include "rangeFinder.h"
#include "tools.h"
#include <libconfig.h++>

namespace SLAM{

//This class models the idea of likelihood fields (probabilistic robotics, ch 6.3). They can be used to compute the probability 
//of a given state with respect to a map and measurement
class LikelihoodField : public MeasurementModel{
public:

	//The parameters vector contains the mixing parameters for the measurement distributions 
	//(see thrun, probabilistic robotics, ch 6.3)
	//Note that zHit + zRandom + zMax = 1 must be fulfilled
	//parameters[0]: sigmaHit: Variance of range measurements
	//parameters[1]: zHit: Influence weight of local measurement noise
	//parameters[2]: zRandom: Influence weight of random failures
	//parameters[3]:  zMax: Influence of max range readings
	//parameters[4]: minOccupiedValue: Minimal value of cell to be viewed as occupied [0,1] 
	//
	//Furthermore an instance rf of a range finder is needed for the likelihood field
	LikelihoodField(std::vector<double> p, RangeFinder* rangeFinder) : MeasurementModel(p), rf(rangeFinder) {};

	//Read parameters from .cfg file. Within this .cfg file there should be a section with the 
	//following structure:
	//	LikelihoodField = {
	//		zHit = ;
	//		sigmaHit = ;
	//		zRandom = ;
	//		zMax = ;
	//		minOccupiedValue = ;
	//	};
	//For parameter explanation see LikelihoodField(std::vector<double> p)
	LikelihoodField (const char* cfgFileName, RangeFinder* rangeFinder);
	
	//Compute the likelihood field of a ranger finder. This function implements the measurement
	//probability. 
	//Algorithm based on Thrun, Probabilistic Robotics,ch 6.4
	//measurement: Vector containing the measurement of each beam of the range finder
	//state: Current state of the robot, where
	//state[0]: x
	//state[1]: y
	//state[2]: orientation
	//map: Map of the environment (should be 2D map of x and y)
	double computeWeight(const std::vector<double>& measurement, const std::vector<double>& state, const Grid<double>& map);


private:
	RangeFinder* rf; //Instance of a range finder
};

}

#endif