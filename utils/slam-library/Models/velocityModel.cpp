#include "velocityModel.h"

namespace SLAM{

VelocityModel::VelocityModel(const char* cfgFileName){
	libconfig::Config cfg;
	cfg.readFile(cfgFileName);
	const libconfig::Setting& root = cfg.getRoot();
	const libconfig::Setting& setting = root["VelocityModel"];
	this->parameters.resize(7);
	setting.lookupValue("alpha1", this->parameters[0]);
	setting.lookupValue("alpha2", this->parameters[1]);
	setting.lookupValue("alpha3", this->parameters[2]);
	setting.lookupValue("alpha4", this->parameters[3]);
	setting.lookupValue("alpha5", this->parameters[4]);
	setting.lookupValue("alpha6", this->parameters[5]);
	setting.lookupValue("dt", this->parameters[6]);
	this->tools = new Tools();
}

std::vector<double> VelocityModel::sample(const std::vector<double>& control, const std::vector<double>& prevState){
	//Algorithm is basic geometry. See Thrun, Probabilistic Robotics,ch 5.3 for a detailed explanation
	double v = control[0];
	double omega = control[1];
	//Add noise
	double vNew = v + this->tools->sampleNormal(this->parameters[0]*v*v + this->parameters[1]*omega*omega);
	double omegaNew = omega + this->tools->sampleNormal(this->parameters[2]*v*v + this->parameters[3]*omega*omega);
	double finalRot = this->tools->sampleNormal(this->parameters[4]*v*v + this->parameters[5]*omega*omega);
	
	std::vector<double> s(3);

	//Predict noisy motion to possible next states
	//'Regular' circular motion
	if(std::abs(omega) > 1e-5){
		s[0] = prevState[0] - vNew/omegaNew * sin(prevState[2]) + vNew/omegaNew * sin(prevState[2] + omegaNew*this->parameters[6]);
		s[1] = prevState[1] + vNew/omegaNew * cos(prevState[2]) - vNew/omegaNew * cos(prevState[2] + omegaNew*this->parameters[6]);
	}
	//Motion on straight line, that is omega -> 0 and thus v/omega -> infty
	else{
		s[0] = prevState[0] + vNew * this->parameters[6] * cos(prevState[2]);
		s[1] = prevState[1] + vNew * this->parameters[6] * sin(prevState[2]);
	}
	s[2] = prevState[2] + omegaNew * this->parameters[6] + finalRot * this->parameters[6];
//	std::cout << omegaNew*this->parameters[6]<<" "<<vNew/omegaNew* sin(prevState[2])<<" "<<vNew/omegaNew * sin(prevState[2] + omegaNew*this->parameters[6])<<std::endl;
//	std::cout << s[0]<<" "<<s[1]<<" "<<s[2]<<std::endl;
	return s;
}
}
