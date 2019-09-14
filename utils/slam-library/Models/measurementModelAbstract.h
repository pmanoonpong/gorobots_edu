#ifndef SB_MEASURE_H_
#define SB_MEASURE_H_

#include <vector>
#include "../grid.h"

namespace SLAM{

//Abstract class to compute weights based on the measurement probabilty distribution. The computation (and thus the underlying model)
//must be implemented by derived classes through the computeWeightmethod
class MeasurementModel{
public:
	//Just copy parameter values
	MeasurementModel(std::vector<double> p) : parameters(p) {};
	
	//Default constructor
	MeasurementModel(){
		this->parameters.clear();
	}
	
	//Computation of weights, which is to be overwritten by derived classes. This method should contain the underlying 
	//measurement model
	virtual double computeWeight(const std::vector<double>& measurement, const std::vector<double>& state, const Grid<double>& map) = 0;
	
	//Vector parameters needed for the motion model. The concrete meaning may vary between different models.
	std::vector<double> parameters;
};

}

#endif
