#include "likelihoodField.h"

namespace SLAM{

LikelihoodField::LikelihoodField (const char* cfgFileName, RangeFinder* rangeFinder) : rf(rangeFinder){
	libconfig::Config cfg;
	cfg.readFile(cfgFileName);
	const libconfig::Setting& root = cfg.getRoot();
	const libconfig::Setting& setting = root["LikelihoodField"];
	this->parameters.resize(5);
	setting.lookupValue("sigmaHit", this->parameters[0]);
	setting.lookupValue("zHit", this->parameters[1]);
	setting.lookupValue("zRandom", this->parameters[2]);
	setting.lookupValue("zMax", this->parameters[3]);
	setting.lookupValue("minOccupiedValue", this->parameters[4]);
}


double LikelihoodField::computeWeight(const std::vector<double>& measurement, const std::vector<double>& state, const Grid<double>& map){
	double q = 1;
	//Get location of range finder in global coordinates
	std::vector<double> locationRF = this->rf->getLocation(state);
	std::vector<double> endPoint(2);
	//Temporary variables
	double z,minDist,dist,sample, beamAngle;
	for(int i = 0; i < (int)measurement.size(); i++){
		//Current measurement
		z = measurement[i];
		beamAngle = this->rf->getBeamAngle(i);
		if (z != this->rf->getErrorValue()){
			//Compute end position of current beam
			endPoint[0] = locationRF[0] + z * cos(beamAngle + state[2]);
			endPoint[1] = locationRF[1] + z * sin(beamAngle + state[2]);
			//Find nearest occupied cell in map
			minDist = 0;
			for(int j = 0; j < map.getNumberCells(); j++){
				Cell<double> c = map.getCell(j);
				if(c.value > this->parameters[4]){
					dist = c.getDistanceToCenter(endPoint);
					if(dist < minDist || minDist == 0)
						minDist = dist;
				}
			}
			//Update probability of measurement
			sample = Tools::probNormal(minDist, this->parameters[0]);
			q = q * (this->parameters[1] * sample + this->parameters[2]/this->parameters[3]);
		}
	}
	return q;
}

}

