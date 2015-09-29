#ifndef SB_OCCMAPBRESEN_H_
#define SB_OCCMAPBRESEN_H_

#include "mapModelAbstract.h"
#include "rangeFinder.h"
#include <libconfig.h++>

namespace SLAM{

//This class models the state space of the robot based on the idea of occupancy grid maps. The state space is divided
//into cells. Each cell is assigned a value in [0,1], which specifies the probability of the cell being occupied by an
//obstacle.
//The update of the map is based on Bresenham's line drawing algorithm, which is used to detect all
//cells between the start and end point of the RF beam
class OccupancyGridBresenham : public MapModel{
public:
	//Information assigned to cells which are believed to be occupied/free
	//Higher values assume higher confidence in measurement and lead to faster
	//convergence of map.
	//parameters[2]: l0
	//parameters[1]: lOcc
	//parameters[0]: lFree
	//
	//Furthermore an instance rf of a range finder is needed for the likelihood field
	OccupancyGridBresenham(std::vector<double> p, RangeFinder* rangeFinder) : MapModel(p), rf(rangeFinder) {};

	//Read parameters from .cfg file. Within this .cfg file there should be a section with the
	//following structure:
	//	OccupancyMap = {
	//		lFree = ;
	//		lOcc = ;
	//		l0 = ;
	//	};
	//For parameter explanation see OccupancyGridMap(std::vector<double> p)
	OccupancyGridBresenham(const char* cfgFileName, RangeFinder* rangeFinder);


	//Update an occupancy grid map based on based on Bresenham's line drawing algorithm, which is used to detect all
	//cells between the start and end point of the RF beam (http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
	//state: Current state of the robot, where
	//state[0]: x
	//state[1]: y
	//state[2]: orientation
	//measurement: Vector containing the measurement of each beam of the range finder
	//map: Map to be updated (will be modified in this function). Should be a 2D map of x,y coordinate
	void updateMap(const std::vector<double>& measurement, const std::vector<double>& state, Grid<double>& map);

private:
	//Computes all cells, which are in a direct line between the start position and the
	//end position. For this, the Bresenham line drawing algorithm is used
	//(http://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C.2B.2B)
	//Returns flat indices of all cells, which are on this line
	//NOTE: Cells of start and end point are NOT in the returned indices list
	std::vector<int> getCellsInLine(const std::vector<double>& start,
			const std::vector<double>& end, const Grid<double>& m);

	RangeFinder* rf; //Instance of a range finder

};

}

#endif
