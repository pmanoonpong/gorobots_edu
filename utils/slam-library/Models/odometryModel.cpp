#include "odometryModel.h"

namespace SLAM{

OdometryModel::OdometryModel(const char* cfgFileName){
	libconfig::Config cfg;
	cfg.readFile(cfgFileName);
	const libconfig::Setting& root = cfg.getRoot();
	const libconfig::Setting& setting = root["OdometryModel"];
	this->parameters.resize(4);
	setting.lookupValue("alpha1", this->parameters[0]);
	setting.lookupValue("alpha2", this->parameters[1]);
	setting.lookupValue("alpha3", this->parameters[2]);
	setting.lookupValue("alpha4", this->parameters[3]);
	this->tools = new Tools(1);
}

std::vector<double> OdometryModel::sample(const std::vector<double>& control, const std::vector<double>& prevState){
	//Algorithm is basic geometry. See Thrun, Probabilistic Robotics,ch 5.4 for a detailed explanation
	double diffX = control[0] - control[3];
	double diffY = control[1] - control[4];

	double deltaRot1 = atan2(diffY, diffX)- control[5];
	double deltaTrans = sqrt(diffX * diffX +  diffY * diffY);
	double deltaRot2 = control[2] - control[5] - deltaRot1;

	double newRot1 = deltaRot1 - this->tools->sampleNormal(this->parameters[0]*deltaRot1*deltaRot1 + this->parameters[1]*deltaTrans*deltaTrans);
	double newTrans = deltaTrans - this->tools->sampleNormal(this->parameters[2]*deltaTrans*deltaTrans +
			this->parameters[3]*(deltaRot1*deltaRot1 + deltaRot2*deltaRot2));
	double newRot2 = deltaRot2 - this->tools->sampleNormal(this->parameters[0]*deltaRot2*deltaRot2 + this->parameters[1]*deltaTrans*deltaTrans);

	std::vector<double> s(3);
	s[0] = prevState[0] + newTrans * cos(prevState[2] + newRot1);
	s[1] = prevState[1] + newTrans * sin(prevState[2] + newRot1);
	s[2] = prevState[2] + newRot1 + newRot2;

	return s;
}
}
