#ifndef SB_ODOMETRY_H_
#define SB_ODOMETRY_H_

#include "motionModelAbstract.h"
#include "tools.h"
#include <libconfig.h++>

namespace SLAM{

//This odometry model models the state transition distribution based on odometry readings (given as control input)
//Algorithm taken from "Probabilistic Robotics,ch 5.4."
class OdometryModel : public MotionModel{
public:
	
	//Noise in the odometry measurements is modeled using a Gaussian distribution. These parameters
	//correspond to the individual variances
	//alpha1 and alpha4 loosely specify the uncertainity for rotations, alpah2 and alpha3 are for
	//translations
	//
	//The parameters must be specified the following way
	//parameters[0]: alpha1
	//parameters[1]: alpha2
	//parameters[2]: alpha3
	//parameters[3]: alpha4
	OdometryModel(std::vector<double> p) : MotionModel(p) {};
	
	//Read parameters from .cfg file. Within this .cfg file there should be a section with the 
	//following structure:
	//	OdometryModel = {
	//		alpha1 = ;
	//		alpha2 = ;
	//		alpha3 = ;
	//		alpha4 = ;
	//	};
	//For parameter explanation see OdometryModel(std::vector<double> p)
	OdometryModel(const char* cfgFileName);
	
	//Subfunction to provide odometry samples based on the control action of
	//the robot. This function implements the state transition probability.
	//Algorithm based on Thrun, Probabilistic Robotics,ch 5.4
	//prevState: Previous state of the robot, where
	//prevState[0]: x
	//prevState[1]: y
	//prevState[2]: orientation
	//The control vector should contain the following values:
	//control[0]: current Odometry reading of x position
	//control[1]: current Odometry reading of y position
	//control[2]: current Odometry reading of orientation
	//control[3]: previous Odometry reading of x position
	//control[4]: previous Odometry reading of y position
	//control[5]: previous Odometry reading of orientation
	std::vector<double> sample(const std::vector<double>& control, const std::vector<double>& prevState);

private:
	//Load toolbox
	Tools* tools;
};

}

#endif
