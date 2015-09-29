#ifndef SB_SCANMATCHING_H_
#define SB_SCANMATCHING_H_

#include "rangeFinder.h"

#include <libconfig.h++>

#include <cmath>
#include <vector>
#include <algorithm>

namespace SLAM{

//This class implements the PSM Scan Matching Algorithm (A. Diosi, L. Kleemann: Fast Laser Scan Matching using Polar Coordinates).
//With this algorithm the state change (that is 2D position and orientation) between two consecutive laser scans can be estimated
//and/or improved
//The state of the robot is assumed to be of the following form
//state[0]: x
//state[1]: y
//state[2]: orientation
class ScanMatcher{
public:
	//Read parameters from .cfg file. Within this .cfg file there should be a section with the
	//following structure:
	//	ScanMatcher = {
	//		maxIterations = ;
	//		maxDifferenceBetweenSegments = ;
	//		maxErrorBetweenScanPoints = ;
	//		weightParameter = ;
	//		maxAccuracy = ;
	//		searchWindow = ;
	//	};
	//For parameter explanation see documentation of private variables
	//An instance of a range finder is given explicitly
	ScanMatcher(RangeFinder* rf, const char* cfgFileName);
	//Range finder is also constructed from cfg file. See rangeFinder.h for further information
	ScanMatcher(const char* cfgFileName);

	//Estimate the state change between two consecutive range measurements based on the PSM Scan Algorithm
	//newMeasurement: Current scan taken with a rangefinder (which is defined through the parameters in RangeFinder rf)
	//				  Note that the order of the scan points must correspond to angles obtainable through
	//				  rf.getBeamAngle(int index)
	//refMeasurement: Reference scan taken with a range finder. The positional change from refMeasurement to newMeasurement
	//				  is estimated/improved
	//newState: Estimate of the state at which newMeasurement was taken. If no estimate is available pass refState.
	//			Note that this vector is updated during this method to include the improved estimates provided
	//			by the Scan Matching Algorithm
	//refState: State of the robot, when refMeasurement was taken. Note that refState and newState must be given in the same
	//			coordinate frame (preferably the global frame), which is not the frame of refState or newState
	void matchScans(std::vector<double> newMeasurement, std::vector<double> refMeasurement,
			std::vector<double>& newState, std::vector<double> refState);

	//Convert newState to the coordinate frame of refState. Both states must be given in the same
	//coordinate frame (preferably the global frame), which is not the frame of refState or newState
	std::vector<double> convertStateToRefFrame(std::vector<double> newState, std::vector<double> refState);
	//Convert state from ref frame to global frame (reverses convertStateToRefFrame)
	std::vector<double> convertRefToGlobal(std::vector<double> state, std::vector<double> refState);

private:
	//During scan matching each scan point is associated with different other variables. All of these associations
	//are tracked with this structure
	struct Range{
		Range();
		Range(double range, double angle, int segIndex, bool tagged);

		double range; //Range of scan point
		double angle; //Angle of scan point
		int segIndex; //Index of segment
		bool tagged; //If tagged is true, scan point will not be used for scan matching
	};

	//Read given config file (used in constructors)
	void readConfig(const char* cfgFileName);
	//Transforms a given scan to the data structure used in this class
	std::vector<Range> registerNewScan(std::vector<double> scan);
	//Preprocess a given scan to increase robustness of scan matching
	void preprocessScan(std::vector<Range>& scan);
	//Median filter with window 5
	void medianFilter5(std::vector<Range>& scan);
	//Estimate how the current scan would like if taken from refState. currState must be given in the
	//coordinate frame of refState
	std::vector<Range> computeTheoreticalScan(std::vector<Range> currScan, std::vector<double> currState);
	//Estimate translation between currScan and refScan based on 'virtual' measurements computed in
	//computeTheoreticalScan
	void estimateTranslation(std::vector<Range> theoScan, std::vector<Range> refScan, double& dx, double& dy);
	//Estimate orientation between currScan and refScan based on 'virtual' measurements computed in
	//computeTheoreticalScan
	void estimateOrientation(std::vector<Range> theoScan, std::vector<Range> refScan, double& dYaw);

	int maxIter; //Maximum number of iterations until scan matching terminates
	double maxDiff; //Maximum difference between two segments in scan
	double maxError; //Maximum error between theoretical and true range at scan point to be considered for state estimation
	double weightParam; //Defines weighting based on error between theoretical and true range
	double maxAccuracy; //Scan matching terminates if change of position is less than this value
	//To estimate orientation between two scans, different shifts are tested to find the one with minimal error
	//This parameter defines how large the maximal shift in this algorithm is
	int searchWindow;

	RangeFinder* rf; //Instance of range finder which provides parameters of range finder
	std::vector<double> stateRF; //State of RF in coordinate system of scan matching algorithm
};
}

#endif
