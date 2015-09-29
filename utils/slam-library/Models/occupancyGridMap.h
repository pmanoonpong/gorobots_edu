#ifndef SB_OCCMAP_H_
#define SB_OCCMAP_H_

#include "tools.h"
#include "mapModelAbstract.h"
#include "rangeFinder.h"
#include <libconfig.h++>

namespace SLAM{

//This class models the state space of the robot based on the idea of occupancy grid maps. The state space is divided
//into cells. Each cell is assigned a value in [0,1], which specifies the probability of the cell being occupied by an
//obstacle
class OccupancyGridMap : public MapModel{
public:
	//The parameters vector should contain the following values
	//parameters[0]: alpha: Width around measured ranges. All cells within this range around an occupied cell
	//						are going to get an occupied value assigned to
	//Note that lFree < l0 < lOcc must be fulfilled 
	//parameters[1]: l0: Information, which is assigned to cell, if measurement carries no information about cell (in logOdds form)
	//Information assigned to cells which are believed to be occupied/free
	//Higher values assume higher confidence in measurement and lead to faster
	//convergence of map
	//parameters[2]: lOcc
	//parameters[3]: lFree
	//
	//Furthermore an instance rf of a range finder is needed for the likelihood field
	OccupancyGridMap(std::vector<double> p, RangeFinder* rangeFinder) : MapModel(p), rf(rangeFinder) {};

	//Read parameters from .cfg file. Within this .cfg file there should be a section with the 
	//following structure:
	//	OccupancyMap = {
	//		alpha = ;
	//		l0 = ;
	//		lFree = ;
	//		lOcc = ;
	//	 	sizeRobot = ;
	//	};
	//For parameter explanation see OccupancyGridMap(std::vector<double> p)
	OccupancyGridMap(const char* cfgFileName, RangeFinder* rangeFinder);

	
	//Update an occupancy grid map based on the measurement and the inverse sensor model.
	//Algorithm based on Thrun, Probabilistic Robotics,ch 9.2
	//state: Current state of the robot, where
	//state[0]: x
	//state[1]: y
	//state[2]: orientation
	//measurement: Vector containing the measurement of each beam of the range finder
	//map: Map to be updated (will be modified in this function). Should be a 2D map of x,y coordinate
	void updateMap(const std::vector<double>& measurement, const std::vector<double>& state, Grid<double>& map);
	
private:
	RangeFinder* rf; //Instance of a range finder
	
	//The inverse sensor model is used to calculate the probability of a map based on a
	//given measurement
	//Algorithm based on Thrun, Probabilistic Robotics,ch 9.2
	//c: Cell for which to compute the measurement probability
	//s: Current state of the robot
	//measurement: Vector containing the measurement of each beam of the range finder
	double inverseSensorModel(const Cell<double>& c, const std::vector<double>& state, const std::vector<double>& measurement);
};

}

#endif
