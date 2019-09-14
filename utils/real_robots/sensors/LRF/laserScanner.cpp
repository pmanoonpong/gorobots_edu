#include "laserScanner.h"

namespace URG{

URGLaser::URGLaser(){
	this->connected = false;
	this->frontBeam = -1;
}

URGLaser::URGLaser(const URGLaser& urg){
	this->frontBeam = urg.frontBeam;
	if(&urg.data != NULL)
		this->data = urg.data;
	for(unsigned int i = 0; i< urg.data.size();i++)
		this->data[i] = urg.data[i];
	const char device[] = "/dev/ttyACM0"; //Device for Linux
	if(urg.connected)
		this->connected = this->urg.connect(device);
	else
		this->connected = false;
}

URGLaser::URGLaser(bool connectLaser){
	//Connect Laser
	if(connectLaser){
		//const char device[] = "COM3"; //Device for Windows
		const char device[] = "/dev/ttyACM0"; //Device for Linux
		//Connect laser
		this->connected = urg.connect(device);
		//Index is shifted because first area_min values were removed during data acquisition
		this->frontBeam = this->urg.parameter().area_front - this->urg.parameter().area_min;
	}
	else{
		this->connected = false;
		this->frontBeam = -1;
	}
}


bool URGLaser::connect(){
	const char device[] = "/dev/ttyACM0"; //Device for Linux
	this->connected = urg.connect(device);
	this->frontBeam = this->urg.parameter().area_front - this->urg.parameter().area_min;
	return this->connected;
}

void URGLaser::disconnect(){
	this->connected = false;
	this->urg.disconnect();
}

bool URGLaser::writeToFile(std::string fileName){
	std::ofstream file;
	bool success = false;
	file.open(fileName.c_str());
	if(file.is_open()){
		success = true;
		for(int i = 0; i < (int)this->data.size(); i++)
			file << data[i] << std::endl;
	}
	file.close();
	return success;
}

int URGLaser::updateData(){
	urg.capture(this->data);
	//For some reasons, data points from 0 to area_min are set to -1 and do not represent real data
	this->data.erase(this->data.begin(),this->data.begin()+this->urg.parameter().area_min);
	return this->data.size();
}

double URGLaser::rad2deg(double rad){
	return 180.0/M_PI*rad;
}

double URGLaser::deg2rad(double degree){
	return M_PI/180.0*degree;
}

//Index has to be shifted because first area min entries were removed during data acquisition
double URGLaser::index2rad(int index){
	return this->urg.index2rad(index+this->urg.parameter().area_min);
}

double URGLaser::index2deg(int index){
	return rad2deg(this->urg.index2rad(index+this->urg.parameter().area_min)) ;
}

int URGLaser::deg2index(double degree){
	return this->urg.rad2index(deg2rad(degree)) - urg.parameter().area_min;
}

double URGLaser::rad2index(double rad){
	return this->urg.rad2index(rad) - urg.parameter().area_min;
}

}
