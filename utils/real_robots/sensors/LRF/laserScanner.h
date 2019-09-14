#ifndef SB_LASERSCANNER_H_
#define SB_LASERSCANNER_H_

//Classes for laser handling
#include "urg/UrgDevice.h"
#include "urg/RangeSensorParameter.h"

#include <fstream>
#include <string>

//using namespace qrk; //Namespace of laser

namespace URG{

class URGLaser{

	int laserDeviation; //Inaccuracy of the laser scanner. This is about 5 - 10 mm and is set within the constructor of the object

public:
	URGLaser(); //Default constructor: Does not connect the laser scanner
	URGLaser(bool connectLaser); //Connects laser scanner and sets its height and angle (should be given in degree) values to the passed ones
	//Copy constructor
	URGLaser(const URGLaser& urg);

	//Connects the laser scanner and sets its height and angle (should be given in degree)
	bool connect();

	//Disconnects the laser
	void disconnect();

	qrk::UrgDevice urg; //Instance of the laser controller
	std::vector<long> data; //Stores last set of data received from the laser
	bool connected; //Indicates if the laser scanner is connected or not. Can be used for error checking
	int frontBeam; //Index of front beam

	
	///<-----All methods below should only be used if the laser is connect---->
	//This has to be checked in top level programs and is not done in the methods themselves

	//Convert degree to rad and vice versa
	static double deg2rad(double degree);
	static double rad2deg(double rad);
	//Convert index to degree/rad and vice versa
	int deg2index(double degree);
	double rad2index(double rad);
	double index2deg(int index);
	double index2rad(int index);

	//Write last measurement of LRF to file
	bool writeToFile(std::string fileName);

	//Updates the data array by requesting new scan data from the laser, returns the length of the data array
	int updateData();
	
};
}
#endif /* SB_LASERSCANNER_H_ */

