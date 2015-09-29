#ifndef SB_MOTION_H_
#define SB_MOTION_H_

#include <vector>

namespace SLAM{

//Abstract class to sample from a given state transition distribution. The sampling (and thus the underlying model)
//must be implemented by derived classes through the sample method
class MotionModel{
public:
	//Just copy parameter values
	MotionModel(std::vector<double> p) : parameters(p) {};
	
	//Default Constructor
	MotionModel(){
		this->parameters.clear();
	}
	
	//Sampling method, which is to be overwritten by derived classes. This method should contain the sampling
	//process and the underlying motion model
	virtual std::vector<double> sample(const std::vector<double>& control, const std::vector<double>& prevState) = 0;
	
	//Vector parameters needed for the motion model. The concrete meaning may vary between different models.
	std::vector<double> parameters;
};

}

#endif