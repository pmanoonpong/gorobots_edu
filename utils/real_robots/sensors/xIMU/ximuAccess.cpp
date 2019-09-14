#include "ximuAccess.h"

namespace XIMU{

Ximu::Ximu(){
	this->init();
}

Ximu::Ximu(const char* cfgFileName){
	this->init();
	if(!this->openConnection()){
		std::cout << "Ximu not connected"<<std::endl;
	}
	int numCalibrate;

	libconfig::Config cfg;
	cfg.readFile(cfgFileName);
	const libconfig::Setting& root = cfg.getRoot();
	const libconfig::Setting& setting = root["XIMU"];

	//First do calibration to obtain offsets (change too much to specify them in file)
	setting.lookupValue("numberCalibrate", numCalibrate);
	this->calibrate(numCalibrate);

	//Then set noise values through file
	setting.lookupValue("noiseAccZ", this->noiseAccZ);
	setting.lookupValue("noiseAccY", this->noiseAccY);
	setting.lookupValue("noiseAccX", this->noiseAccX);
	setting.lookupValue("noiseGyrZ", this->noiseGyrZ);
	setting.lookupValue("noiseGyrY", this->noiseGyrY);
	setting.lookupValue("noiseGyrX", this->noiseGyrX);

	setting.lookupValue("maxSizeData", this->maxSizeData);
	setting.lookupValue("weightLowPass", this->weightLowPass);
}

void Ximu::init(){
	this->isConnected_ = false;	
	this->isReadingData_ = false;
	this->doPause = false;
	this->sleepForMS = 100;
	//Set maxSizeData to default value
	this->maxSizeData = 1e6;
	//No rotation in the beginning
	this->currentOrientation = Rotation3D();
	//Default start position is origin without velocity
	this->position.resize(3);
	this->velocity.resize(6);
	//Default measure rate in Hz
	this->measureRate = 235.;//256.;
	//Parameters for low pass and complementary filter
	this->weightComplementaryFilter = 0.02;
	this->weightLowPass = 1.;
	//No offsets per default. Might be set through calibrate method
	this->offSetMagnet  = 0;
	this->offSetGyrX = this->noiseGyrX = 0;
	this->offSetGyrY = this->noiseGyrY = 0;
	this->offSetGyrZ = this->noiseGyrZ = 0;
	this->offSetAccX = this->noiseAccX = 0;
	this->offSetAccY = this->noiseAccY = 0;
	this->offSetAccZ = this->noiseAccZ = 0;
	this->oldAccX = 0;
	this->oldAccY = 0;
	this->oldAccZ = 0;
}

Ximu::~Ximu(){
	//Stop reading thread when destroying this object and then disconnect
	this->disconnect();
}

std::vector<double> Ximu::getPosition(){
	return this->position;
}

std::vector<double> Ximu::getVelocity(){
	return this->velocity;
}

std::vector<double> Ximu::getAcceleration(){
	std::vector<double> acc(3);
	int index = (int)this->accData.size()-1;
	acc[0] = this->accData[index].x;
	acc[1] = this->accData[index].y;
	acc[2] = this->accData[index].z;
	return acc;
}

bool Ximu::openConnection(){
	FT_STATUS ftStatus;
	ftStatus = FT_Open(0, &this->ftHandle);
	this->isConnected_ = ftStatus == FT_OK;
	//If connection attempt was successful, set connection characteristics
	if(this->isConnected_){
		//Set baud rate to standard value
		ftStatus=FT_SetBaudRate(ftHandle, FT_BAUD_115200);
		//Set data characteristics to standard values (8-Bit per Byte, 1 stop bit and 
		//no parity bit)
		ftStatus=FT_SetDataCharacteristics(ftHandle,FT_BITS_8,FT_STOP_BITS_1,FT_PARITY_NONE);
		//Set device and data terminal to not ready to exchange data
		ftStatus=FT_ClrDtr(ftHandle);
		ftStatus=FT_ClrRts(ftHandle);
	}
	return this->isConnected_;
}

void Ximu::printType(){
	DWORD deviceID;
	FT_DEVICE ftDevice;
	FT_STATUS ftStatus;
	char SerialNumber[16];
	char Description[64];
	ftStatus = FT_GetDeviceInfo(this->ftHandle,&ftDevice,&deviceID,SerialNumber,Description,NULL);
	if (ftStatus == FT_OK) {
		if (ftDevice == FT_DEVICE_232H)
			std::cout << "FT232H"<<std::endl; // device is FT232
		else if (ftDevice == FT_DEVICE_4232H)
			std::cout << "FT4232H"<<std::endl; // device is FT4232H
		else if (ftDevice == FT_DEVICE_2232H)
			std::cout << "FT2232H"<<std::endl; // device is FT
		else if (ftDevice == FT_DEVICE_232R)
			std::cout << "FT232R"<<std::endl; // device is FT232R
		else if (ftDevice == FT_DEVICE_2232C)
			std::cout << "FT2232C"<<std::endl; // device is FT2232C/L/D
		else if (ftDevice == FT_DEVICE_BM)
			std::cout << "FTU232BM"<<std::endl; // device is FTU232BM
		else if (ftDevice == FT_DEVICE_AM)
			std::cout << "FT8U232AM"<<std::endl; // device is FT8U232AM
		else
			std::cout << "Unknown Device"<<std::endl;; // unknown device (this should not happen!)
	}
	else
		std::cout << "Device not connected"<<std::endl;
}

void Ximu::disconnect(){
	//Stop thread, if still running
	if(this->isReadingData_)
		this->stopReading();
	FT_Close(this->ftHandle);
	this->isConnected_ = false;
}

bool Ximu::isConnected(){
	return this->isConnected_;
}

bool Ximu::isReadingData(){
	return this->isReadingData_;
}

void Ximu::getOrientation(Rotation3D& orientation){
	orientation = this->currentOrientation;
}
void Ximu::getOrientation(double& yaw, double& pitch, double& roll){
	this->currentOrientation.getEuler(yaw, pitch, roll);
}

void Ximu::estimateOrientation(InertialAndMagStruct dataStruct, double& yaw, double& pitch, double& roll){
	//Compute roll and pitch angles from acceleration data. Idea is to relate measured accelerations to gravity.
	//This only yields reasonable results, if no external forces are applied at the moment.
	//A complementary filter can be used to compensate this in combination with gyroscope data
	double accX = dataStruct.accX;
	double accY = dataStruct.accY;
	double accZ = dataStruct.accZ;
	//http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
	roll= atan2(accY, accZ);
//	pitch= atan2(dataStruct.accX, dataStruct.accZ);
	//TODO: Check if this works too
	//pitch = atan2(-accX,sqrt(accY*accY + accZ*accZ));
	pitch = atan(-accX/sqrt(accY*accY + accZ*accZ));
	double magX = dataStruct.magX;
	double magY = dataStruct.magY;
	double magZ = dataStruct.magZ;
	//Yaw angle cannot be computed from acceleration data. Thus use Magnetometer measurements to estimate yaw angle
	//Note that this value might have an offset. Use calibrateMagnetometer() to deal with this
	//http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf
	double x = magX * cos(pitch) + magY*sin(pitch)*sin(roll) + magZ*sin(pitch)*cos(roll);
	double y = magZ * sin(roll) - magY*cos(roll);
	yaw = atan2(-y,x);
	//Convert angle to range [0, 2pi] to be able to compensate offset
	if(yaw < 0)
		yaw += 2*M_PI;
}

void Ximu::update(InertialAndMagStruct& dataStruct){
	//Preprocess data by including offsets and smoothin acceleration values
	this->preprocessData(dataStruct);
	//Update orientation of xIMU based on current measurement
	this->updateOrientation(dataStruct);
	//Update position and velocity. Rotates acceleration values in dataStruct to global frame
	this->updatePosition(dataStruct);

	//Log measurement
	Triple dataPoint;
	this->orientations.push_back(this->currentOrientation);
	dataPoint = Triple(dataStruct.accX, dataStruct.accY, dataStruct.accZ);
	this->accData.push_back(dataPoint);
	dataPoint = Triple(dataStruct.gyrX, dataStruct.gyrY, dataStruct.gyrZ);
	this->gyrData.push_back(dataPoint);
	dataPoint = Triple(dataStruct.magX, dataStruct.magY, dataStruct.magZ);
	this->magData.push_back(dataPoint);
}


void Ximu::updatePosition(InertialAndMagStruct& dataStruct){
	//Convert from g to m/s^2
	double accX = (double)dataStruct.accX * 9.81;
	double accY = (double)dataStruct.accY * 9.81;
	double accZ = (double)dataStruct.accZ * 9.81;

	//Time interval between two measurements
	double dt = 1./this->measureRate;
	//Integrate velocity
	double vX = accX * dt + this->velocity[0];
	double vY = accY * dt + this->velocity[1];
	double vZ = accZ * dt + this->velocity[2];
	//Double integrate positions
	double x = 0.5 * accX * dt * dt + this->velocity[0] * dt + this->position[0];
	double y = 0.5 * accY * dt * dt + this->velocity[1] * dt + this->position[1];
	double z = 0.5 * accZ * dt * dt + this->velocity[2] * dt + this->position[2];
	//Update member variables
	this->position[0] = x;
	this->position[1] = y;
	this->position[2] = z;
	
	this->velocity[0] = vX;
	this->velocity[1] = vY;
	this->velocity[2] = vZ;
	this->velocity[3] = dataStruct.gyrX;
	this->velocity[4] = dataStruct.gyrY;
	this->velocity[5] = dataStruct.gyrZ;
}

void Ximu::updateOrientation(InertialAndMagStruct& dataStruct){
	//Time between two measurements
	double dt = 1./this->measureRate;
	//Get angles velocities from data structure, integrate them based on measure rate
	//and convert them to rad
	double roll = dt * dataStruct.gyrX * M_PI/180.;
	double pitch = dt * dataStruct.gyrY * M_PI/180.;
	double yaw = dt * dataStruct.gyrZ * M_PI/180.;
	//Create rotation
	Rotation3D rot = Rotation3D(yaw, pitch, roll);
	//Update current rotations
	//TODO: Maybe multiplication order should be inversed (not sure)
	this->currentOrientation = rot * this->currentOrientation;
	this->currentOrientation.normalize();
	//Use acceleration information to improve pose estimate (complementary filter0
	this->currentOrientation.getEuler(yaw, pitch, roll);
	double rollAcc, pitchAcc, yawMag;
	//Get orientation estimate based on accelerometer and magnetometer
	this->estimateOrientation(dataStruct,yawMag, pitchAcc, rollAcc);
	//Compensate offset of magnetometer
	yawMag -= this->offSetMagnet;
	//Combine information
	roll = (1 - this->weightComplementaryFilter) * roll + this->weightComplementaryFilter * rollAcc;
	pitch = (1 - this->weightComplementaryFilter) * pitch + this->weightComplementaryFilter * pitchAcc;
	//TODO: Magnetometer output is wrong (magnetic field in physics department is not homogenous enough)
	//yaw = (1 - epsilon) * yaw + epsilon * yawMag;

	//Update orientation
	this->currentOrientation.setEuler(yaw, pitch, roll);
}

bool Ximu::startReading(){
	//Check if device is connected and not already reading data
	if(!this->isConnected_ || this->isReadingData_)
		return false;
	//Clear old data
	this->accData.clear();
	this->gyrData.clear();
	this->magData.clear();
	this->orientations.clear();
	this->isReadingData_ = true;
	//Make sure buffer is empty
	this->emptyBuffer();
	//Create and start reading thread
	this->threadIMU = boost::thread(&Ximu::readingThread,this);
	return true;
}

void Ximu::pause(int ms){
	this->sleepForMS = ms;
	this->doPause = true;
}

void Ximu::resetVelocity(){
//	std::fill(this->velocity.begin(), this->velocity.end(), 0);
	this->velocity[0] = 0;
	this->velocity[1] = 0;
	this->velocity[2] = 0;
}

bool Ximu::emptyBuffer(){
	//Temp variables
	FT_STATUS ftStatus;
	DWORD dwRxSize;
	DWORD EventDWord;
	unsigned char buffer[256];
	//Check amount of data in buffer
	ftStatus = FT_GetQueueStatus(this->ftHandle, &dwRxSize);
	//Prepare to read
	ftStatus=FT_SetDtr(this->ftHandle);
	ftStatus=FT_SetRts(this->ftHandle);
	//Read data
	ftStatus = FT_Read(this->ftHandle, &buffer, dwRxSize, &EventDWord);
	//Disable RTS and DTS signal
	ftStatus=FT_ClrDtr(this->ftHandle);
	ftStatus=FT_ClrRts(this->ftHandle);	
	return ftStatus == FT_OK;
}

void Ximu::preprocessData(InertialAndMagStruct& dataStruct){
	//Substract offsets from gyroscope values
	dataStruct.gyrX -= this->offSetGyrX;
	dataStruct.gyrY -= this->offSetGyrY;
	dataStruct.gyrZ -= this->offSetGyrZ;

	//Set to zero if signal is not above noise level
	if(std::abs(dataStruct.gyrX) < this->noiseGyrX)
		dataStruct.gyrX = 0;
	if(std::abs(dataStruct.gyrY) < this->noiseGyrY)
		dataStruct.gyrY = 0;
	if(std::abs(dataStruct.gyrZ) < this->noiseGyrZ)
		dataStruct.gyrZ = 0;

	//Filter acceleration values with low pass and apply offsets
	dataStruct.accX = this->weightLowPass * (dataStruct.accX - this->offSetAccX) + (1 - this->weightLowPass) * this->oldAccX;
	dataStruct.accY = this->weightLowPass * (dataStruct.accY - this->offSetAccY) + (1 - this->weightLowPass) * this->oldAccY;
	dataStruct.accZ = this->weightLowPass * (dataStruct.accZ - this->offSetAccZ) + (1 - this->weightLowPass) * this->oldAccZ;

	//Set to zero if signal is not above noise level
	if(std::abs(dataStruct.accX) < this->noiseAccX)
		dataStruct.accX = 0;
	if(std::abs(dataStruct.accY) < this->noiseAccY)
		dataStruct.accY = 0;
	if(std::abs(dataStruct.accZ) < this->noiseAccZ)
		dataStruct.accZ = 0;

	//Update old values
	this->oldAccX = dataStruct.accX;
	this->oldAccY = dataStruct.accY;
	this->oldAccZ = dataStruct.accZ;
}

bool Ximu::calibrate(int numberDataPoints){
	//Check if properly connected
	if(this->isReadingData_ || !this->isConnected_)
		return false;

	bool bSkip = false;
	//Array to hold data
	unsigned char Buf[256];
	//Bytes actually read
	int bytesRead;
	//Estimates for yaw, pitch and roll angle based on magnetometer and accelerometer
	double rollAcc, pitchAcc, yawMag;
	//Counts number of measurements
	int count = 0;
	//Compute offsets and min/max values
	double sumYaw = 0;
	std::vector<double> gyrX(numberDataPoints);
	std::vector<double> gyrY(numberDataPoints);
	std::vector<double> gyrZ(numberDataPoints);
	std::vector<double> accX(numberDataPoints);
	std::vector<double> accY(numberDataPoints);
	std::vector<double> accZ(numberDataPoints);
	InertialAndMagStruct dataStruct;
	
	//First empty buffer of device
	this->emptyBuffer();
	//Loop until enough data points
	while(count < numberDataPoints){
		//Try readings
		if(!this->readDevice(Buf, 256, bytesRead))
			continue;

		//Process data
		for(int j = 0; j < bytesRead; j++)
			if(this->processNewChar(Buf[j]) != ERR_NO_ERROR){
				bSkip = true;
				break;
			}
		if(bSkip){
			bSkip = false;
			continue;
		}
		//Store processed data
		if(this->inertialAndMagGetReady){
			count++;
			dataStruct = this->getInertialAndMag();
			//First measurement is usually not usable
			if(count != 1){
				gyrX[count - 1] = dataStruct.gyrX;
				gyrY[count - 1] = dataStruct.gyrY;
				gyrZ[count - 1] = dataStruct.gyrZ;
				accX[count - 1] = dataStruct.accX;
				accY[count - 1] = dataStruct.accY;
				accZ[count - 1] = dataStruct.accZ;
				this->estimateOrientation(dataStruct,yawMag, pitchAcc, rollAcc);
				sumYaw += yawMag;
			}
		}
	}
	//Set offset to average measurement
	this->offSetGyrX = std::accumulate(gyrX.begin(),gyrX.end(),0.)/(double)count;
	//Set noise value
	this->noiseGyrX = std::min(std::abs(*std::min_element(gyrX.begin(), gyrX.end()) - this->offSetGyrX),
			std::abs(*std::max_element(gyrX.begin(), gyrX.end()) - this->offSetGyrX));

	this->offSetGyrY = std::accumulate(gyrY.begin(),gyrY.end(),0.)/(double)count;
	this->noiseGyrY = std::min(std::abs(*std::min_element(gyrY.begin(), gyrY.end()) - this->offSetGyrY),
			std::abs(*std::max_element(gyrY.begin(), gyrY.end()) - this->offSetGyrY));

	this->offSetGyrZ = std::accumulate(gyrZ.begin(),gyrZ.end(),0.)/(double)count;
	this->noiseGyrZ = std::min(std::abs(*std::min_element(gyrZ.begin(), gyrZ.end()) - this->offSetGyrZ),
			std::abs(*std::max_element(gyrZ.begin(), gyrZ.end()) - this->offSetGyrZ));

	this->offSetAccX = std::accumulate(accX.begin(),accX.end(),0.)/(double)count;
	this->noiseAccX = std::min(std::abs(*std::min_element(accX.begin(), accX.end()) - this->offSetAccX),
			std::abs(*std::max_element(accX.begin(), accX.end()) - this->offSetAccX));

	this->offSetAccY = std::accumulate(accY.begin(),accY.end(),0.)/(double)count;
	this->noiseAccY = std::min(std::abs(*std::min_element(accY.begin(), accY.end()) - this->offSetGyrX),
			std::abs(*std::max_element(accY.begin(), accY.end()) - this->offSetAccX));

	this->offSetAccZ = std::accumulate(accZ.begin(),accZ.end(),0.)/(double)count;
	this->noiseAccZ = std::min(std::abs(*std::min_element(accZ.begin(), accZ.end()) - this->offSetAccZ),
			std::abs(*std::max_element(accZ.begin(), accZ.end()) - this->offSetAccZ));

	this->offSetMagnet = sumYaw/count;

//	std::cout << this->offSetGyrX<<" "<<this->offSetGyrY<<" "<<this->offSetGyrZ<<std::endl;
//	std::cout << this->offSetAccX<<" "<<this->offSetAccY<<" "<<this->offSetAccZ<<std::endl;

	return true;
}

bool Ximu::stopReading(){
	//Interrupt reading thread
	this->threadIMU.interrupt();
	//Make sure DTR and RTS signals are set to disabled
	FT_STATUS ftStatus;
	ftStatus=FT_ClrDtr(this->ftHandle);
	ftStatus=FT_ClrRts(this->ftHandle);
	this->isReadingData_ = false;
	return ftStatus == FT_OK;
}

bool Ximu::readDevice(unsigned char* chars, unsigned int size, int& bytesRead){
	DWORD dwRxSize = 0;
	//How many bytes are read in practice
	DWORD EventDWord;
	//Status variable of FTDI device
	FT_STATUS ftStatus;
	//Check buffer
	ftStatus = FT_GetQueueStatus(this->ftHandle, &dwRxSize);
	//Too much data in buffer (should not happen)
	if(dwRxSize > size){
		std::cout << "Buffer overflow"<<std::endl;
		return false;
	}
	//Enable RTS and DTS signal
	ftStatus=FT_SetRts(this->ftHandle);
	ftStatus=FT_SetDtr(this->ftHandle);
	//Read data from chip
	ftStatus = FT_Read(this->ftHandle, chars, dwRxSize, &EventDWord);
	//Disable RTS and DTS signal
	ftStatus=FT_ClrDtr(this->ftHandle);
	ftStatus=FT_ClrRts(this->ftHandle);
	bytesRead = dwRxSize;
	return ftStatus == FT_OK;
}

void Ximu::readingThread(){
	//Counts obtained data points
	int count = 0;
	//Structure to hold to measured data
	InertialAndMagStruct dataStruct;
	//Array to hold raw data obtained from the chip
	unsigned char Buf[256];
	int bytesRead;
	//Loop until enough data is obtained or thread is cancelled
	while(count < this->maxSizeData){	
		//Read data from device
		if(!this->readDevice(Buf, 256, bytesRead))
			continue;
		//Convert data to sensory information
		for(int i = 0; i < bytesRead; i++){
			if(this->processNewChar(Buf[i]) != ERR_NO_ERROR)
				std::cout <<"Error processing IMU data"<<std::endl;
		}
		//Stored obtained data
		if(this->inertialAndMagGetReady){
			count++;
			dataStruct = this->getInertialAndMag();
			//First measurement is usually corrupted due to garbage bytes in buffer, thus skip it
			//Usually enough measurements are taken, such that this has no influence
			if(count != 1)
				//Update all position and orientation of xIMU
				this->update(dataStruct);
		}
		//Pause thread
		if(this->doPause){
			boost::this_thread::sleep(boost::posix_time::milliseconds(this->sleepForMS));
			this->doPause = false;
		}
		//Set interruption point to be able to cancel thread
		boost::this_thread::interruption_point() ;
	}
	//If maxSize is reached, update isReading flag
	this->isReadingData_ = false;
}


bool Ximu::writeDataToFile(std::string fileName){
	if(this->isReadingData_)
		return false;
	std::ofstream file;
	file.open(fileName.c_str());
	if(file.is_open()){
		file <<"#Index\taccX\taccY\taccZ\tgyrX\tgyrY\tgyrZ"<<std::endl;
		for(unsigned int i = 0; i < this->gyrData.size(); i++){
			//Write acceleration data
			file <<i<<"\t"<< this->accData[i].x << "\t" <<this->accData[i].y << "\t"<<
					this->accData[i].z << "\t";
			//Write gyro data
			file << this->gyrData[i].x << "\t" <<this->gyrData[i].y << "\t"<<
					this->gyrData[i].z << std::endl;
		}
	}
	else
		return false;
	return true;
}

}
