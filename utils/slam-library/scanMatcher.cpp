#include "scanMatcher.h"

namespace SLAM{

ScanMatcher::ScanMatcher(RangeFinder* rf, const char* cfgFileName){
	this->rf = rf;
	this->readConfig(cfgFileName);
}

ScanMatcher::ScanMatcher(const char* cfgFileName){
	this->rf = new RangeFinder(cfgFileName);
	this->readConfig(cfgFileName);
}

void ScanMatcher::readConfig(const char* cfgFileName){
	libconfig::Config cfg;
	cfg.readFile(cfgFileName);
	const libconfig::Setting& root = cfg.getRoot();
	const libconfig::Setting& rfSetting = root["ScanMatcher"];
	rfSetting.lookupValue("maxIterations", this->maxIter);
	rfSetting.lookupValue("maxDifferenceBetweenSegments", this->maxDiff);
	rfSetting.lookupValue("maxErrorBetweenScanPoints", this->maxError);
	rfSetting.lookupValue("maxAccuracy", this->maxAccuracy);
	rfSetting.lookupValue("weightParameter", this->weightParam);
	rfSetting.lookupValue("searchWindow", this->searchWindow);

	//Compute state of RF
	//TODO: should not be done here but in seperate method
	this->stateRF.resize(3);
	std::vector<double> posRF = this->rf->getRelativeDisplacement();
	this->stateRF[0] = posRF[0];
	this->stateRF[1] = posRF[1];
	//'Virtual' rotation of range finder to fit coordinate system of scan matching algorithm. This means, that
	//first scan is taken at 0 deg and corresponds to x axis of range finder
	int sign = 1;
	if(this->rf->getEndAngle() < this->rf->getStartAngle())
		sign = -1;
	this->stateRF[2] = sign * this->rf->getStartAngle();
}

std::vector<ScanMatcher::Range> ScanMatcher::registerNewScan(std::vector<double> scan){
	int n = (int)scan.size();
	std::vector<Range> ranges(n);
	Range r;

	//Get start and angle
	double startAngle = this->rf->getStartAngle();
	double endAngle = this->rf->getEndAngle();

	//The scan matching algorithm assumes measurements to be taken in increasing
	//angular order. This means that the angle of beam i must be greater than the angle of beam i-1.
	//Furthermore, the first measurement must correspond to an angle of zero.
	//The following lines transform the incoming measurement of the rangefinder to fulfill these properties.

	//Check if angles are already in increasing order
	if(endAngle > startAngle){
		//Store ranges in normal order
		for(int i = 0; i < n; i++){
			r.range = scan[i];
			//Measurements must start at 0 deg
			r.angle = this->rf->getBeamAngle(i) - startAngle;
			if(scan[i] == this->rf->getErrorValue())
				r.tagged = true;
			ranges[i] = r;
		}
	}
	else{
		for(int i = n - 1; i >= 0; i--){
			r.range = scan[i];
			//Measurements must start at 0 deg
			r.angle = this->rf->getBeamAngle(i) - endAngle;
			if(scan[i] == this->rf->getErrorValue())
				r.tagged = true;
			//Store ranges in reverse order to ensure increasing angles between beams
			ranges[n - i - 1] = r;
		}
	}
	return ranges;
}

std::vector<double> ScanMatcher::convertStateToRefFrame(std::vector<double> newState, std::vector<double> refState){
	std::vector<double> state(3);
	//Compute transformation to coordinate frame of refState
	double beta = this->stateRF[2] + refState[2] - newState[2];
	double gamma = this->stateRF[2] + refState[2];

	state[0] = this->stateRF[0] * (cos(beta) - cos(this->stateRF[2])) + this->stateRF[1] * (sin(beta) - sin(this->stateRF[2])) +
			(newState[0] - refState[0]) * cos(gamma) + (newState[1] - refState[1]) * sin(gamma);
	state[1] = -this->stateRF[0] * (sin(beta) - sin(this->stateRF[2])) + this->stateRF[1] * (cos(beta) - cos(this->stateRF[2])) -
			(newState[0] - refState[0]) * sin(gamma) + (newState[1] - refState[1]) * cos(gamma);
	state[2] = newState[2] - refState[2];

	return state;

}

std::vector<double> ScanMatcher::convertRefToGlobal(std::vector<double> state, std::vector<double> refState){
	std::vector<double> newState(3);

	double x1 = state[0] * cos(this->stateRF[2]) - state[1] * sin(this->stateRF[2]) + this->stateRF[0];
	double y1 = state[0] * sin(this->stateRF[2]) + state[1] * cos(this->stateRF[2]) + this->stateRF[1];

	newState[0] = x1 * cos(refState[2]) - y1 * sin(refState[2]) + refState[0];
	newState[1] = x1 * sin(refState[2]) + y1 * cos(refState[2]) + refState[1];
	newState[2] = state[2] + refState[2];

	return newState;
}

void ScanMatcher::preprocessScan(std::vector<Range>& scan){
	int n = scan.size();
	std::vector<int> segLength;

	//Apply median filter
	this->medianFilter5(scan);

	//Divide laser scan into segments
	int currSegLength = 1; //Length of current segment
	for(int i = 1; i < n; i++){
		if(std::abs(scan[i].range - scan[i-1].range) > this->maxDiff || i == n-1){
			//Make sure last point is dealt with in a proper way
			if(i == n-1)
				currSegLength++;
			segLength.push_back(currSegLength);
			currSegLength = 0;
		}
		currSegLength++;
	}

	//Assign unique number to each point in each segment
	//0 is used for segments of length 1. They are not considered in scan matching
	int index = 0;
	int segIndex = 1;
	for(int i = 0; i < (int)segLength.size(); i++){
		if(segLength[i] == 1){
			scan[index].segIndex = 0;
			index++;
		}
		else{
			for(int j = 0; j < segLength[i]; j++){
				scan[index].segIndex = segIndex;
				index++;
			}
			segIndex++;
		}
	}
}

void ScanMatcher::medianFilter5(std::vector<Range>& scan){
	std::vector<double> tempRange(5);
	for(int i = 2; i < (int)scan.size() - 2; i++){
		for(int j = -2; j < 3; j++)
			tempRange[j+2] = scan[i + j].range;

		std::sort(tempRange.begin(), tempRange.end());
		scan[i].range = tempRange[2];
	}
}

std::vector<ScanMatcher::Range> ScanMatcher::computeTheoreticalScan(std::vector<Range> currScan, std::vector<double> currState){
	//First project scan to reference position, that is compute how the scan would have looked, if taken from refState
	int n = (int)currScan.size();
	double beamAngleStep = std::abs(this->rf->getBeamAngleStep());
	//Compute current scan as would be seen from refState
	std::vector<Range> scanFromRef(n);
	for(int i = 0; i < n; i++){
		double x = currScan[i].range * cos(currState[2] + currScan[i].angle) + currState[0];
		double y = currScan[i].range * sin(currState[2] + currScan[i].angle) + currState[1];
//		double angle = atan2(y,x);
//		if(angle < 0)
//			angle += 2 * M_PI;
		Range r(sqrt(x*x + y*y), atan2(y,x), currScan[i].segIndex, false);
		scanFromRef[i] = r;
	}

	//Now interpolate between projected scan points to obtain 'virtual' measurements at possible beam angles of the RF
	std::vector<Range> theoScan(n); //Range are set to high value per default (see constructor Range)
	double a0, a1, j0, j1, r0, r1;
	bool occluded = false;
	double beamAngle = beamAngleStep;
	theoScan[0].angle = 0;
	for(int i = 1; i < n; i++){
		//Set beam angles
		theoScan[i].angle = beamAngle;
		beamAngle += beamAngleStep;
		//Check if both scan points are valid for scan matching (same segment, positive angles)
		if(scanFromRef[i].angle <= 0 || scanFromRef[i-1].angle < 0 || scanFromRef[i].segIndex == 0 ||
				scanFromRef[i].segIndex != scanFromRef[i-1].segIndex)
			continue;
		//Check order of angles and thus, if scan point is occluded by other points
		if(scanFromRef[i].angle > scanFromRef[i-1].angle){
			occluded = false;
			a0 = scanFromRef[i-1].angle;
			a1 = scanFromRef[i].angle;
			j0 = ceil(scanFromRef[i-1].angle / beamAngleStep);
			j1 = floor(scanFromRef[i].angle / beamAngleStep);
			r0 = scanFromRef[i-1].range;
			r1 = scanFromRef[i].range;
		}
		else{
			occluded = true;
			a0 = scanFromRef[i].angle;
			a1 = scanFromRef[i-1].angle;
			j0 = ceil(scanFromRef[i].angle / beamAngleStep);
			j1 = floor(scanFromRef[i-1].angle / beamAngleStep);
			r0 = scanFromRef[i].range;
			r1 = scanFromRef[i-1].range;
		}
		//Compute theoretical measurement at 'real' angle by interpolation
		while(j0 <= j1){
			double r = (r1 - r0) / (a1 - a0) * (j0 * beamAngleStep - a0) +r0;
			if(j0 >= 0 && j0 < n && theoScan[j0].range > r){
				theoScan[j0].range = r;
				theoScan[j0].tagged = occluded;
			}
			j0++;
		}
	}

	return theoScan;
}


void ScanMatcher::estimateTranslation(std::vector<Range> theoScan, std::vector<Range> refScan, double& dx, double& dy){
	//Temp variables for 'manual' matrix multiplication
	double hwr1 = 0;
	double hwr2 = 0;
	double hwh11 = 0;
	double hwh12 = 0;
	double hwh21 = 0;
	double hwh22 = 0;
	//Compute matrix which defines translation estimate (2x2)
	for(int i = 0; i < (int)theoScan.size(); i++){
		double deltaR = refScan[i].range - theoScan[i].range;
		if(!theoScan[i].tagged && std::abs(deltaR) < this->maxError){
			double w = this->weightParam / (deltaR*deltaR + this->weightParam);
			double h1 = cos(theoScan[i].angle);
			double h2 = sin(theoScan[i].angle);
			hwr1 += w * h1 * deltaR;
			hwr2 += w * h2 * deltaR;
			hwh11 += w* h1 * h1;
			hwh22 += w* h2 * h2;
			hwh12 += w * h1 * h2;
			hwh21 += w * h1 * h2;
		}
	}
	//Manually compute inverse
	double d = hwh11 * hwh22 - hwh12*hwh21;
	double inv11 = hwh22/d;
	double inv12 = -hwh12/d;
	double inv21 = -hwh21/d;
	double inv22 = hwh11/d;
	//Compute translation estimate based on matrix
	dx = inv11 * hwr1 + inv12*hwr2;
	dy = inv21 * hwr1 + inv22*hwr2;
}

void ScanMatcher::estimateOrientation(std::vector<Range> theoScan, std::vector<Range> refScan, double& dYaw){
	int k = 0;
	std::vector<double> error(2*this->searchWindow);
	std::vector<double> beta(2*this->searchWindow);
	//Loop through different shifts of scan to find the one which fits best
	for(int deltaI = -this->searchWindow; deltaI <= this->searchWindow; deltaI++){
		int n = 0;
		double e = 0;
		int min_i, max_i;
		//Compute correct indices for looping through array
		if(deltaI <= 0){
			min_i = -deltaI;
			max_i = (int)theoScan.size();
		}
		else{
			min_i = 0;
			max_i = (int)theoScan.size() - deltaI;
		}
		//Compute mean error
		for(int i = min_i; i < max_i; i++){
			if(!theoScan[i].tagged){
				e += std::abs(theoScan[i].range - refScan[i+deltaI].range);
				n++;
			}
		}
		if(n > 0)
			error[k] = e/(double)n;
		else
			error[k] = 1e6;
		//Store error and shift
		beta[k] = deltaI;
		k++;
	}

	//Interpolate minimal error through quadratic interpolation
	double eMin = 1e6;
	int indexMin = -1;
	//First find minimum
	for(int i = 0; i < k; i++){
		if(error[i] < eMin){
			eMin = error[i];
			indexMin = i;
		}
	}
	//Then find minimum of parabola given through eMin and points right and left of it
	double m = (error[indexMin + 1] - error[indexMin - 1]) / (2 * (2 * error[indexMin] - error[indexMin - 1] - error[indexMin + 1]));
	dYaw = (beta[indexMin] + m) * std::abs(this->rf->getBeamAngleStep());
}

void ScanMatcher::matchScans(std::vector<double> newMeasurement, std::vector<double> refMeasurement,
		std::vector<double>& newState, std::vector<double> refState){
	int iter = 0;
	double dx, dy, dYaw;
	std::vector<Range> theoScan;
	//Transform scans to data structure necessary for scan matching algorithm
	std::vector<Range> currScan = this->registerNewScan(newMeasurement);
//	for(int i = 0 ; i < (int)currScan.size();i++)
//		std::cout << i << " "<<currScan[i].angle *180./M_PI<<" "<<currScan[i].range<<std::endl;
	std::vector<Range> refScan = this->registerNewScan(refMeasurement);
	//Transform current (global) state to coordinate frame of reference state
	std::vector<double> currState = this->convertStateToRefFrame(newState, refState);
	//Prepare scan for scan matching
	this->preprocessScan(currScan);
	//First estimate translation, then based on obtained values compute orientation estimate
	while(iter < this->maxIter){
		theoScan = this->computeTheoreticalScan(currScan, currState);
		this->estimateTranslation(theoScan, refScan, dx, dy);
		currState[0] += dx;
		currState[1] += dy;
		iter++;
	}
	iter = 0;
	while(iter < this->maxIter){
		theoScan = this->computeTheoreticalScan(currScan, currState);
		this->estimateOrientation(theoScan, refScan, dYaw);
		currState[2] += dYaw;
		iter++;
	}
	newState = this->convertRefToGlobal(currState, refState);
}

ScanMatcher::Range::Range(){
	range = 1e6;
	segIndex = 0;
	tagged = true;
	angle = 2.*M_PI;
}
ScanMatcher::Range::Range(double range, double angle, int segIndex, bool tagged){
	this->range = range;
	this->segIndex = segIndex;
	this->tagged = tagged;
	this->angle = angle;
}

}
