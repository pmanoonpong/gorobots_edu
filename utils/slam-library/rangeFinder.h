#ifndef SB_ABSTRACTRF_H_
#define SB_ABSTRACTRF_H_

#include "grid.h"
#include <vector>
#include <libconfig.h++>

namespace SLAM{

//TODO: Add virtual method to query data from real range finder. In this way, for each real sensor a class
//can be derived which overwrites this virtual method

//Class to represent a range finder and its parameters. This class is intended as a base class for
//range finders and may be extended to represent different range finders, in particular their way to obtain data.
//Angle convention: Two angles are given as parameters to the constructor of this class (combined in a vector).
//The first angle is the angle of the first beam of the range finder (index = 0), the second angle is the angle
//of the last beam (index = numberBeams). The angles are assumed to be given with respect to the movement direction
//of the robot, e.g. a measurement at angle zero is assumed to point in the forward direction of the robot.
//The intermediate beams are assumed to be spaced evenly across the range. Counter clock-wise rotations correspond to positive angles.
//All angles are handled in rad.
//
//The state of the robot is assumed to be of the following form
//state[0]: x
//state[1]: y
//state[2]: orientation
class RangeFinder{
public:
	//Standard constructor. For meaning of individual parameters, see documentation of private variables
	RangeFinder(std::vector<double> angles, int numberBeams, double maxRange, int errorValue ,
				double beamWidth, std::vector<double> displacement);
	
	//Read parameters from .cfg file. Within this .cfg file there should be a section with the 
	//following structure:
	//	RangeFinder ={
	//		angles = {
	//			startAngle = ;
	//			endAngle = ;
	//		};
	//		numberBeams = ;
	//		maxRange = ;
	//		beamWidth = ;
	//		position = {x = ; y = ;};
	//		errorValue = ;
	//	};
	//For parameter explanation see documentation of private variables
	RangeFinder(const char* cfgFileName);
	
	//Get maximum range of range finder
	virtual double getMaxRange();
	//Get beam width of range finder
	virtual double getBeamWidth();
	//Get angle the beam specified by index (index = 0 corresponds to the starting beam, that is
	//the beam with angle of angles[0])
	virtual double getBeamAngle(int index);
	//Easy access to start/end angle
	virtual double getStartAngle();
	virtual double getEndAngle();
	//Get difference between two beam angles
	virtual double getBeamAngleStep();
	//Return displacement of RF
	virtual std::vector<double> getRelativeDisplacement();
	//Get error value of range finder, that is, value which is returned on faulty measurements
	virtual int getErrorValue();
	//Get position of range finder in global coordinates. For this the current state of the robot is needed.
	virtual std::vector<double> getLocation(const std::vector<double>& stateRobot);
	//Check if cell is in perceptual field of range finder, that is, its distance is less than max range.
	//Note that, cells, which are slightly further away than maxRange, will also be accepted
	virtual bool isInPerceptField(const Cell<double>& c, const std::vector<double>& stateRobot);
	//Get index of the beam, whose angle is closest to the given angle phi.
	virtual int getIndexNearestBeam(double phi);

protected:
	//Error value of the range finder. This is the value, the range finder returns for faulty measurements
	int errorValue; 
	//Angle of first(angles[0]) and last(angles[1) beam of the range finder. Angles are given in rad.
	//Angles are assumed to be given with respect to the movement direction of the robot, e.g. a measurement at angle
	//zero is assumed to point in the forward direction of the robot.
	//Note that positive angles correspond to counter clockwise rotations
	std::vector<double> angles;
	//Total number of beams
	int numberBeams; 
	//Maximum range of range finder
	double maxRange;
	//Width of an individual beam of the range finder (in rad)
	double beamWidth; 
	//Angle difference between two consecutive beams
	double stepBeamAngle;
	//Vector containing all beam angles for fast look up
	//Beam i has angle beamAngles[i]. In particular beamAngles[0] = angles[0] and beamAngles[numberBeams-1] = angles[1]
	std::vector<double> beamAngles;
	//Relative location of the range finder with respect to the position of the robot (in most cases center of mass)
	std::vector<double> displacement;
	//Generate all beam angles and assign them to beamAngles
	void generateBeamAngles();
};

}
#endif

