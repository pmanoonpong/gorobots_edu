#ifndef SB_MAPMODEL_H_
#define SB_MAPMODEL_H_

#include <vector>
#include "../grid.h"

namespace SLAM{

//Abstract class to update a map based on the measurements and the state of the robot. The computation (and thus the underlying model)
//must be implemented by derived classes through the updateMap method
class MapModel{
public:
	//Just copy parameter values
	MapModel(std::vector<double> p) : parameters(p) {};
	
	//Default constructor
	MapModel(){
		this->parameters.clear();
	}
	
	//Update of map, which is to be overwritten by derived classes. This method should contain the underlying 
	//map model
	virtual void updateMap(const std::vector<double>& measurement, const std::vector<double>& state, Grid<double>& map) = 0;
	
	//Vector parameters needed for the motion model. The concrete meaning may vary between different models.
	std::vector<double> parameters;
};

}

#endif
