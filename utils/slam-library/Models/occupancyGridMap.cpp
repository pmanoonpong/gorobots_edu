#include "occupancyGridMap.h"

namespace SLAM{

OccupancyGridMap::OccupancyGridMap(const char* cfgFileName, RangeFinder* rangeFinder) : rf(rangeFinder){
	libconfig::Config cfg;
	cfg.readFile(cfgFileName);
	const libconfig::Setting& root = cfg.getRoot();
	const libconfig::Setting& setting = root["OccupancyMap"];
	this->parameters.resize(4);
	setting.lookupValue("alpha", this->parameters[0]);
	setting.lookupValue("l0", this->parameters[1]);
	setting.lookupValue("lFree", this->parameters[3]);
	setting.lookupValue("lOcc", this->parameters[2]);
	setting.lookupValue("sizeRobot", this->parameters[4]);
}

void OccupancyGridMap::updateMap(const std::vector<double>& measurement, const std::vector<double>& state, Grid<double>& map){
	double l,prob;
	Cell<double> c;
	//Loop through all cells
	for(int i = 0; i < map.getNumberCells(); i++){
		c = map.getCell(i);
		//Check if cell is in perceptual field of range finder
		if(this->rf->isInPerceptField(c, state)){
			//Compute log odds
			l = log(c.value / (1 - c.value));
			//Update cell through inverse model
			l += this->inverseSensorModel(c, state, measurement) - this->parameters[1];
			//Convert back to probability
			prob = 1 - 1 / (1 + exp(l));
			//Update map
			map.setCellValue(i, prob);
		}
	}
}

double OccupancyGridMap::inverseSensorModel(const Cell<double>& c, const std::vector<double>& state, const std::vector<double>& measurement){
	//Compute global location of range finder
	std::vector<double> locationRF = this->rf->getLocation(state);
	std::vector<double> centerCell = c.getCenter();
	//Compute distance and angle from position of range finder to cell
	double diffX = centerCell[0] - locationRF[0];
	double diffY = centerCell[1] - locationRF[1];
	double r = sqrt(diffX*diffX + diffY*diffY);
	//If cell is very close to robot, cell must be free (otherwise robot would not be there)
	if(r <= this->parameters[4])
		return this->parameters[3];
	//Make sure angle is in [-pi,pi]
	double phi = Tools::wrapAngle(atan2(diffY, diffX) - state[2]);
	//Select beam nearest to cell
	int k = this->rf->getIndexNearestBeam(phi);
	//Corresponding measurement of beam
	double z = measurement[k];
	//Check if cell is outside of the range of the sensor or if faulty measurement. In this case, measurement carries no information
	//except the prior knowledge about cell occupation (coded in l0)
	if( r >= std::min(this->rf->getMaxRange(), z + this->parameters[0]/2.) || z == this->rf->getErrorValue() ||
			std::abs(phi - this->rf->getBeamAngle(k)) > this->rf->getBeamWidth()/2.){
		return this->parameters[1];
	}
	//Cell is in a small block with width alpha around real measurement. In this case the cell is assumed
	//to be occupied
	else if(z < this->rf->getMaxRange() && std::abs(r - z) < this->parameters[0]/2.)
		return this->parameters[2];
	//Cell is between obstacle (measurement) and robot. In this case, the cell is assumed to be free.
	//Note that measurements with tag '-1' are 'max range measurements. All cells in this beam are assumed
	//to be free either. For this, the sensor has to provide different values for faulty measurements and max range measurement
	else if(r <= z)
		return this->parameters[3];
	//Should not happen
	else
		return NAN;

}
}
