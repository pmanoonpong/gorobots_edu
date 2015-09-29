#ifndef SB_XIMU_H_
#define SB_XIMU_H_

#include "XimuReceiver.h"
#include "Rotation3D.h"
#include "ftd2xx.h"

#include <numeric>
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <libconfig.h++>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace XIMU{

//Triple to store measurements (or data in general) in three dimensions
struct Triple{
	Triple(){
		x = y = z = NAN;
	}
	
	Triple(double x, double y, double z){
		this->x = x;
		this->y = y;
		this->z = z;
	}
	
	double x;
	double y;
	double z;
};

//Class to provide easy access to the x-IMU. Threaded data acquisition and high level functions 
//are handled in this class. The conversion of the raw data is done in XimuReceiver.cpp (obtained from
//https://github.com/xioTechnologies/x-IMU-Arduino-Example).
//The angle conventions used in this class correspond to the standard aeronautical representation, that is:
//yaw: Rotation around z-axis (z-axis is oriented downwards)
//pitch: Rotation around y-axis
//roll: Orientation around x-axis
//All angles are right handed.
class Ximu : XimuReceiver{
public:
	//Standard constructor. Does NOT connect the device. MaxSizeData is set to 1e6
	Ximu();
	//Define parameters (offsets, noise levels, ...) through parameter file. Will automatically
	//connect and calibrate device
	Ximu(const char* cfgFileName);
	//Own destructor is necessary to properly close the device and existing threads
	~Ximu();
	//Opens a connection to the x-IMU. This is necessary before being able to read data
	//Returns if attempt was successfull (can later be queried by isConnected())
	bool openConnection();
	//Disconnects the device (and ends possible threads, which read data)
	void disconnect();
	//Start reading data from the device. For this a separate thread is created, such that
	//the data can be obtained without blocking the calling thread. 
	//At the beginning of this method, all old data is cleared.
	//Data reading stops automatically, if maxSizeData samples were obtained.
	bool startReading();
	//Explicitly stop data acquistion
	bool stopReading();
	//Pause reading thread for given number of milliseconds
	void pause(int ms);
	//Check, if device is connected
	bool isConnected();
	//Check, if device is currently reading data
	bool isReadingData();
	//Print the type of the FTDI chip
	void printType();
	//Write obtained data to a given file.
	//This function will not write any data, if device is currently reading data, that is
	//if isReadingData returns true.
	bool writeDataToFile(std::string fileName);

	//Maximum number of data points acquired from the x-IMU (specifies max size of accData)
	int maxSizeData;
	//Get current position of xIMU (x,y,z)
	std::vector<double> getPosition();
	//Get current velocity of xIMU (vX, vY, vZ, vPitch, vRoll, vYaw)
	std::vector<double> getVelocity();
	//Get current acceleration
	std::vector<double> getAcceleration();
	//Get current orientation of xIMU in quaternion representation
	void getOrientation(Rotation3D& orientation);
	//Get current orientation in euler angles (see Rotation3D.h for further information)
	//Angles are returned in rad
	void getOrientation(double& yaw, double& pitch, double& roll);
	//The output of the magnetometer is used in combination with the gyroscope values to estimate
	//the yaw angle of the xIMU. As the magnetometer returns absolute orientation values (with respect
	//to the magnetic field of the earth), it is necessary to tell the xIMU, which values corresponds
	//to a yaw angle of zero. 
	//In this method a certain number of measurements are carried out. The average orientation output of the
	//magnetometer is computed and set as offset.
	//Furthermore constant offsets for all raw measurements are computed by averaging over all output values
	//IMPORTANT: During the calibration the xIMU must not be moved. It should be positioned, such that its orientation
	//corresponds to a yaw angle of zero and its z-axis is in line with gravitation.
	bool calibrate(int numberDataPoints);
	//Reset velocity to zero
	void resetVelocity();
	
	//Measure rate of xIMU in Hz. Default value is 256 Hz. In most cases its value should not be changed.
	//IMPORTANT: Changing this value will NOT change the actual measure rate of the device
	double measureRate;
	//Weight for complementary filter, which is used to combine gyroscope and accelerometer/magnetometer values
	double weightComplementaryFilter;
	//Weight for low pass filter, which smoothes acceleration values
	double weightLowPass;
	
	//Acceleration data obtained from the x-IMU. Is cleared automatically when startReading()
	//is called. Maximum size of this vector is given by maxSizeData
	std::vector<Rotation3D> orientations; //in quaternions
	std::vector<Triple> accData; //in g
	std::vector<Triple> gyrData; //in deg/sec	
	std::vector<Triple> magData; //in Ga
		
private:
	//Perform a complete update of the pose (orientation and position) of the xIMU
	void update(InertialAndMagStruct& dataStruct);
	//Preprocess data by applying offsets and low-pass-filtering the acceleration values
	void preprocessData(InertialAndMagStruct& dataStruct);
	//Update the current orientation of the xIMU based on given measurement.
	//This method uses a complementary filter to include acceleration and magnetic information. This compensates the
	//drift induced by the integration of the gyroscope data.
	//NOTE: Use unrotated acceleration values in this method (always call before updatePosition)
	void updateOrientation(InertialAndMagStruct& dataStruct);
	//Updates position and velocity of xIMU based on acceleration values. Rotates acceleration values in dataStruct
	//to global frame and removes influence of gravitation
	void updatePosition(InertialAndMagStruct& dataStruct);
	//All standard initialization stuff is in here. Can be called by constructors
	void init();
	//Use the data of the accelerometer and the magnetometer to estimate the current pose of the xIMU. Note that in contrast
	//to other methods, the yaw angle is returned within the range [0,2pi]
	//The gyroscope data is NOT used in this method.
	void estimateOrientation(InertialAndMagStruct dataStruct, double& yaw, double& pitch, double& roll);
	//Read data from the device
	//chars: Array to hold the received data
	//size: Size of chars
	//bytesRead: Number of bytes that were actually read (can later be used to loop through data)
	bool readDevice(unsigned char* chars, unsigned int size, int& bytesRead);
	//Empty the buffer of the device, that is read all available data and discard it
	bool emptyBuffer();
	//Function, which is executed in an individual thread to read data from the IMU
	void readingThread();	
	
	//Current position of xIMU
	std::vector<double> position;
	//Current velocity
	std::vector<double> velocity;
	//Current orientation of xIMU
	Rotation3D currentOrientation;
	//Offset for  magnetometer
	double offSetMagnet;
	//Offset and min/max values gyroscope
	double offSetGyrX, noiseGyrX;
	double offSetGyrY, noiseGyrY;
	double offSetGyrZ, noiseGyrZ;
	//Offset and min/max values accelerometer
	double offSetAccX, noiseAccX;
	double offSetAccY, noiseAccY;
	double offSetAccZ, noiseAccZ;
	//Old acceleration values. Used for low-pass-filtering
	double oldAccX;
	double oldAccY;
	double oldAccZ;
	//Check if device is connected
	bool isConnected_;
	//Check if device is reading
	bool isReadingData_;
	//Check if reading thread should be paused (and for how long)
	bool doPause;
	int sleepForMS;
	//Thread in which to read data
	boost::thread threadIMU;
	//Instance of the device itself
	FT_HANDLE ftHandle;
};


}
#endif
