#ifndef SB_SLAMSOLVER_H_
#define SB_SLAMSOLVER_H_

#include "tools.h"
#include "grid.h"
#include "rangeFinder.h"
#include "particle.h"
#include "Models/measurementModelAbstract.h"
#include "Models/mapModelAbstract.h"
#include "Models/motionModelAbstract.h"

#include <boost/math/distributions/normal.hpp>
#include <libconfig.h++>

#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>

namespace SLAM{

//Structure containing all parameters needed for KLD sampling (Thrun, Probabilistic Robotics,ch 8.3.7). 
//Note that this structure also includes a grid representation of the state space and thus, might need
//a lot of memory.
struct KLDParameters{
	//Standard constructor
	//epsilon, sigma and minNumber: See description of variables
	//lowerBounds, upperBounds, resolutions: These values desribe the size and accuracy of the grid representation.
	KLDParameters(double epsilon, double sigma, double minNumber, 
			std::vector<double> lowerBounds, std::vector<double> upperBounds, std::vector<double> resolutions){
		this->epsilon = epsilon;
		this->sigma = sigma;
		this->minNumberParticles = minNumber;
		//Init state space representation (true = cell is occupied, false = cell is free)
		this->gridStateSpace = Grid<bool>(lowerBounds, upperBounds, resolutions, false);
		//Init boost normal distribution, which is needed to compute quantiles later one
		this->normalDistribution = boost::math::normal(0.0, 1.0);
	}
	
	//Read parameters from .cfg file. Within this .cfg file there should be a section with the 
	//following structure:
	//	KLD = {
	//		lowerBounds = [, , ... , ];
	//		upperBounds = [, , ... , ];
	//		resolutions = [, , ... , ];
	//		epsilon = ;
	//		sigma = ;
	//		minNumberParticles = ;
	//	};
	//For parameter explanation see description of variables
	KLDParameters(const char* cfgFileName){
		libconfig::Config cfg;
		cfg.readFile(cfgFileName);
		const libconfig::Setting& root = cfg.getRoot();

		const libconfig::Setting& kldSetting = root["KLD"];
		kldSetting.lookupValue("epsilon", this->epsilon);
		kldSetting.lookupValue("sigma", this->sigma);
		kldSetting.lookupValue("minNumberParticles", this->minNumberParticles);

		const libconfig::Setting& lbSetting = root["KLD"]["lowerBounds"];
		std::vector<double> lowerBounds(lbSetting.getLength());
		for(unsigned int i = 0 ; i < lowerBounds.size(); i++)
			lowerBounds[i] = lbSetting[i];
		
		const libconfig::Setting& ubSetting = root["KLD"]["upperBounds"];
		std::vector<double> upperBounds(ubSetting.getLength());
		for(unsigned int i = 0 ; i < upperBounds.size(); i++)
			upperBounds[i] = ubSetting[i];
		
		const libconfig::Setting& resSetting = root["KLD"]["resolutions"];
		std::vector<double> resolutions (resSetting.getLength());
		for(unsigned int i = 0 ; i < resolutions.size(); i++)
			resolutions[i] = resSetting[i];
		
		this->gridStateSpace = Grid<bool>(lowerBounds, upperBounds, resolutions, false);
		//Init boost normal distribution, which is needed to compute quantiles later one
		this->normalDistribution = boost::math::normal(0.0, 1.0);
	}
	
	KLDParameters(){
		this->epsilon = this->sigma = this->minNumberParticles = -1;
		//Empty grid
		gridStateSpace = Grid<bool>();
	}
	//Maximal error between approximation and true posterior
	double epsilon;
	//1 - sigma is the probability, that the error of the approximation is less than epsilon 
	double sigma;
	//Minimal number of particles.
	int minNumberParticles;
	//Grid representation of the state space
	Grid<bool> gridStateSpace;
	//Boost normal distribution, which is needed to compute quantiles later one
	boost::math::normal normalDistribution;
};


//Class representing all parameters and functions associated with a robot. In particular, localization and mapping algorithms 
//can be found in this class.
class SLAMSolver{
public:
	//Default constructor (Nothing will be initialized)
	SLAMSolver(){};
	
	//Standard constructor without using KLD.
	//Init belief: The intial belief of the state of the robot, represented by particles
	//(The particles should be distributed according to the initial belief distribution)
	//rf: Structure of parameters, which characterize the range finder
	//param: Structure of parameters, which define the measurement model
	//Note that this function basically calls init() and sets the KLD parameter to false
	SLAMSolver(std::vector<Particle> initBelief, MeasurementModel* measureModel, MotionModel* motionModel, MapModel* mapModel);
	
	//Standard constructor with KLD enabled
	//Init belief: The intial belief of the state of the robot, represented by particles
	//(The particles should be distributed according to the initial belief distribution)
	//rf: Structure of parameters, which characterize the range finder
	//param: Structure of parameters, which define the measurement model
	//kldParam: Structure of parameters, which are used for KLD sampling 
	//Note that this function basically calls init() and sets the KLD parameter to true
	SLAMSolver(std::vector<Particle> initBelief, MeasurementModel* measureModel, MotionModel* motionModel, MapModel* mapModel,
			KLDParameters* kldParam);
	
	//FastSLAM step for occupancy grid maps. 
	//Algorithm based on Thrun, Probabilistic Robotics,ch 13.10
	//prevOdometry: Odometry measurement of the previous timestep
	//currOdometry: Odometry measurement of the current timestep
	//measurement: Vector containing measurements by all beams of a range finder
	//Note that this function is only a wrapper function for particleFilterNaive and particleFilterKLD.
	void fastSlamStep(const std::vector<double>& control, const std::vector<double>& measurement);
	
	//Monte Carlo localization step. This function assumes that a correct map of the environment is
	//available
	//Algorithm based on Thrun, Probabilistic Robotics,ch 8.3
	//prevOdometry: Odometry measurement of the previous timestep
	//currOdometry: Odometry measurement of the current timestep
	//measurement: Vector containing measurements by all beams of a range finder
	//m: Correct map of the environment
	//Depending on the value of useKLD, the naive implementation without KLD sampling or the extended version
	//with KLD sampling is used.
	//Note that this function is only a wrapper function for particleFilterNaive and particleFilterKLD.
	void MCL_step(const std::vector<double>& control, const std::vector<double>& measurement, const Grid<double>& m);

	//Return the particles, which represent the current belief distribution of the robot
	std::vector<Particle> getBelief();
	
	//Write the particles of the belief distribution to a file, e.g. to be able to plot them
	//Note that only the state of the particle is written to the file. The corresponding map is omitted
	void writeBelief(std::string fileName);
	
	//Check if KLD sampling is enabled
	bool isUsingKLD();
	//Disable KLD sampling
	void disableKLD();
	//Enable KLD sampling and provide necessary parameters through structure.
	void enableKLD(KLDParameters* kldParam);
	
private:
	//Load toolbox
	Tools* tools;

	//Measurement model used by the particle filter
	MeasurementModel* measureModel;
	//Motion model used by the particle filter
	MotionModel* motionModel;
	//Map model used by the particle filter
	MapModel* mapModel;
	
	//Initialization method, which basically assign the given values to the corresponding members
	//Should be used by constructors
	void init(std::vector<Particle> initBelief, MeasurementModel* measureModel, MotionModel* motionModel, MapModel* mapModel);
	
	//Naive implementation of a particle filter. The parameter doSLAM determines if the particle filter is applied to the SLAMSolver
	//problem or the MCL problem.
	//doSLAM: Determines if particle filter is applied to the SLAMSolver problem (true) or MCL problem (false)
	//		  Note that, if this parameter is true, the given map will be ignored
	//m: Correct map of the environment (is ignored for the SLAMSolver problem, that is doSLAM is true)
	//Other parameters as in MCL_step
	//Algorithm based on Thrun, Probabilistic Robotics,ch 8.3.2 and ch. 13.10
	void particleFilterNaive(bool doSLAM, const std::vector<double>& control, const std::vector<double>& measurement, 
			const Grid<double>& m);
	//Advanced implementation of a particle filter using KLD sampling. The parameter doSLAM determines if the particle filter is 
	//applied to the SLAMSolver problem or the MCL problem.
	//doSLAM: Determines if particle filter is applied to the SLAMSolver problem (true) or MCL problem (false)
	//		  Note that, if this parameter is true, the given map will be ignored
	//m: Correct map of the environment (is ignored for the SLAMSolver problem, that is doSLAM is true)
	//Other parameters as in MCL_step
	//Algorithm based on Thrun, Probabilistic Robotics,ch 8.3.7
	void particleFilterKLD(bool doSLAM, const std::vector<double>& control, const std::vector<double>& measurement, const Grid<double>& m);
	
	//Vector, containing the particles which represent the current belief distribution
	std::vector<Particle> currentBelief;
	//Current number of particles
	int numberParticles;
	//This structure defines the parameters used for KLD sampling. Will be ignored if useKLD is false
	KLDParameters* kldParam;
	//If this flag is true, KLD sampling will be used for MCL and FastSLAM
	bool useKLD;
	
	//Vector containing the weight of each particle (used for faster and easier sampling with drawWeights)
	//Note that this parameter is only used in methods with KLD sampling. 
	//When using this value, make sure that it corresponds to the current belief. It is NOT updated automatically.
	std::vector<double> weightsBelief;
	//Sum of all weights of particles
	//Note that this parameter is only used in methods with KLD sampling (used for faster and easier sampling with drawWeights).
	//When using this value, make sure that it corresponds to the current belief. It is NOT updated automatically.
	double sumWeights;
};

}

#endif
