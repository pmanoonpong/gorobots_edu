#ifndef SB_VELOCITY_H_
#define SB_VELOCITY_H_

#include "motionModelAbstract.h"
#include "tools.h"
#include <libconfig.h++>

namespace SLAM{

//This odometry model models the state transition distribution based on
//velocity readings (given as control input)
//Algorithm taken from "Probabilistic Robotics,ch 5.3"
class VelocityModel : public MotionModel{
public:
	
	//Noise in the velocity is modeled using a Gaussian distribution. These parameters
	//correspond to the individual variances
	//alpha1 and alpha2: Translation
	//alpha2 and alpah4: Rotation
	//alpah5 and alpha6: (Final rotation)
	//Furthermore the time deltaT between two measurements is needed. During this time the motion is assumed to be constant.
	//
	//The parameters must be specified the following way
	//parameters[0]: alpha1
	//parameters[1]: alpha2
	//parameters[2]: alpha3
	//parameters[3]: alpha4
	//parameters[4]: alpha5
	//parameters[5]: alpha6
	//parameters[6]: time step deltaT
	VelocityModel(std::vector<double> p) : MotionModel(p) {};
	
	//Read parameters from .cfg file. Within this .cfg file there should be a section with the 
	//following structure:
	//	VelocityModel = {
	//		alpha1 = ;
	//		alpha2 = ;
	//		alpha3 = ;
	//		alpha4 = ;
	//		alpha5 = ;
	//		alpha6 = ;
	//		dt =;
	//	};
	//For parameter explanation see OdometryModel(std::vector<double> p)
	VelocityModel(const char* cfgFileName);
	
	//Subfunction to provide odometry samples based on the control action of
	//the robot. This function implements the state transition probability.
	//Algorithm based on Thrun, Probabilistic Robotics,ch 5.3
	//prevState: Previous state of the robot, where
	//prevState[0]: x
	//prevState[1]: y
	//prevState[2]: orientation
	//The control vector should contain the following values:
	//control[0]: current translation velocity in direction of current orientation of robot (forward is positive)
	//control[1]: current rotational velocity around z axis of robot (yaw angle) (counter clockwise is positive)
	std::vector<double> sample(const std::vector<double>& control, const std::vector<double>& prevState);

private:
	//Load toolbox
	Tools* tools;
};

}

#endif
