#include "rangeFinder.h"

namespace SLAM{

RangeFinder::RangeFinder(std::vector<double> angles, int numberBeams, double maxRange, int errorValue ,
		double beamWidth, std::vector<double> displacement){
	this->angles = angles;
	this->numberBeams = numberBeams;
	this->maxRange = maxRange;
	this->errorValue = errorValue;
	this->beamWidth = beamWidth;
	this->displacement = displacement;
	this->stepBeamAngle = (this->angles[1] - this->angles[0])/(this->numberBeams - 1);
	this->generateBeamAngles();
}

RangeFinder::RangeFinder(const char* cfgFileName){
	libconfig::Config cfg;
	cfg.readFile(cfgFileName);
	const libconfig::Setting& root = cfg.getRoot();
	
	const libconfig::Setting& rfSetting = root["RangeFinder"];
	rfSetting.lookupValue("numberBeams", this->numberBeams);
	rfSetting.lookupValue("maxRange", this->maxRange);
	rfSetting.lookupValue("errorValue", this->errorValue);
	rfSetting.lookupValue("beamWidth", this->beamWidth);
	
	const libconfig::Setting& position = root["RangeFinder"]["position"];
	this->displacement.resize(2);
	position.lookupValue("x", this->displacement[0]);
	position.lookupValue("y", this->displacement[1]);
	
	const libconfig::Setting& anglesSetting = root["RangeFinder"]["angles"];
	this->angles.resize(2);
	anglesSetting.lookupValue("startAngle", this->angles[0]);
	anglesSetting.lookupValue("endAngle", this->angles[1]);
	
	this->stepBeamAngle = (this->angles[1] - this->angles[0])/(this->numberBeams - 1);
	this->generateBeamAngles();
}

bool RangeFinder::isInPerceptField(const Cell<double>& c, const std::vector<double>& stateRobot){
	std::vector<double> rfLoc = this->getLocation(stateRobot);
	double dist = c.getDistanceToCenter(rfLoc);
	//Also accept cells which are slightly further away (inverseModel might apply to them)
	return dist <= this->maxRange + dist/10.;
}

int RangeFinder::getIndexNearestBeam(double phi){
	int index = -1;
	double minDiff = -1;
	for(int i = 0; i < this->numberBeams; i++){
		double diff = std::abs(phi - this->getBeamAngle(i));
		if(minDiff == -1 || diff < minDiff){
			minDiff = diff;
			index = i;
		}
	}	
	return index;
}

//For illustration and algorithm see Thrun, Probabilistic Robotics,ch 6.4
std::vector<double> RangeFinder::getLocation(const std::vector<double>& stateRobot){
	std::vector<double> location(2);
	location[0] = stateRobot[0] + cos(stateRobot[2])*this->displacement[0] - sin(stateRobot[2])*this->displacement[1];
	location[1]= stateRobot[1] + sin(stateRobot[2])*this->displacement[0] + cos(stateRobot[2])*this->displacement[1];
	return location;
}

void RangeFinder::generateBeamAngles(){
	this->beamAngles.resize(this->numberBeams);
	for(int i = 0; i < this->numberBeams; i++)
		this->beamAngles[i] = this->angles[0] + i*this->stepBeamAngle;

}

int RangeFinder::getErrorValue(){
	return this->errorValue;
}

double RangeFinder::getMaxRange(){
	return this->maxRange;
}

double RangeFinder::getBeamWidth(){
	return this->beamWidth;
}

double RangeFinder::getBeamAngle(int index){
	return this->beamAngles[index];
}

double RangeFinder::getStartAngle(){
	return this->angles[0];
}

double RangeFinder::getEndAngle(){
	return this->angles[1];
}

std::vector<double> RangeFinder::getRelativeDisplacement(){
	return this->displacement;
}

double RangeFinder::getBeamAngleStep(){
	return this->stepBeamAngle;
}

}

