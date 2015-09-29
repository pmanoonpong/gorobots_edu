#include "ximuAccess.h" //Class to provide access to xIMU
#include "Rotation3D.h" //Quaternions

#include <iostream>
#include <fstream>
#include <unistd.h>

using namespace std;
using namespace XIMU;

int main(int argc, char *argv[])
{
	Ximu device;
	//Try to connect to xIMU
	if(!device.openConnection()){
		cout << "Device not connected. Did you use sudo?"<<endl;
		return -1;
	}

	int sleepMilli = 500; //Time between two queries to IMU
	//Start calibration of IMU with 500 samples. This is useful to compensate for
	//possible offsets of the measurements
	cout << "Start calibration"<<endl;
	device.calibrate(500);
	cout << "End calibration"<<endl;
	//The IMU acquires its measurements in its own thread and stores/integrates them
	//The thread will continue running until stopped
	device.startReading();
	//Number of samples to be taken
	int maxSize = 1000;
	for(int i = 0; (int)device.accData.size() < maxSize; i++){
		usleep(sleepMilli * 1000);
		//Print progess
		cout << (double)device.gyrData.size()/(double)maxSize * 100.<<"%"<<endl;
	}
	//Stop measurement thread of IMU
	device.stopReading();
	//Write logged data to file
	device.writeDataToFile("testXIMU.dat");
	//Print final orientation
	double yaw, pitch,roll;
	device.getOrientation(yaw,pitch, roll);
	cout << "Final orientation: "<<endl;
	cout << yaw *180./M_PI<< " "<< pitch*180./M_PI << " " <<roll*180./M_PI<<" "<<device.gyrData.size() <<endl;
}
	
