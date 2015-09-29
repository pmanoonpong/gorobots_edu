#include "occupancyGridBresenham.h"

namespace SLAM{

OccupancyGridBresenham::OccupancyGridBresenham(const char* cfgFileName, RangeFinder* rangeFinder) : rf(rangeFinder){
	libconfig::Config cfg;
	cfg.readFile(cfgFileName);
	const libconfig::Setting& root = cfg.getRoot();
	const libconfig::Setting& setting = root["OccupancyMap"];
	this->parameters.resize(3);
	setting.lookupValue("lFree", this->parameters[0]);
	setting.lookupValue("lOcc", this->parameters[1]);
	setting.lookupValue("l0", this->parameters[2]);
}

void OccupancyGridBresenham::updateMap(const std::vector<double>& measurement, const std::vector<double>& state, Grid<double>& map){
	//Location of range finder
	std::vector<double> locationRF = this->rf->getLocation(state);
	std::vector<double> endPoint(2);
	double beamAngle;
	double cellValue = 0;
	double l,prob;

	//First set starting point of beam to free (as the robot is at this position)
	//(done in logOdds representation)
	if(map.getCellValue(locationRF, cellValue)){
		l = log(cellValue / (1 - cellValue));
		l += this->parameters[0] + this->parameters[2];
		//Convert back to probability
		prob = 1 - 1 / (1 + exp(l));
		//Update map
		map.setCellValue(locationRF, prob);
	}

	//Loop through all measurements
	for(int i = 0; i < (int)measurement.size(); i++){
		double z = measurement[i];
		beamAngle = this->rf->getBeamAngle(i);
		//Compute end position of current beam
		endPoint[0] = locationRF[0] + z * cos(beamAngle + state[2]);
		endPoint[1] = locationRF[1] + z * sin(beamAngle + state[2]);
		//Check for valid measurement
		if (z != this->rf->getErrorValue()){
			//Set cell of end point to occupied, if object was measured (that is no
			//max range measurement)
			if(z < this->rf->getMaxRange()){
				if(map.getCellValue(endPoint, cellValue)){
					l = log(cellValue / (1 - cellValue));
					//Update cell
					l += this->parameters[1] + this->parameters[2];
					//Convert back to probability
					prob = 1 - 1 / (1 + exp(l));
					//Update map
					map.setCellValue(endPoint, prob);
				}
			}
			//Get all cells, which are traversed by RF beam.
			std::vector<int> lineIndices = this->getCellsInLine(locationRF, endPoint, map);
			//Set these cells to free, as beam did not meet any obstacles
			for(unsigned j = 0; j < lineIndices.size(); j++){
				map.getCellValue(lineIndices[j], cellValue);
				l = log(cellValue / (1 - cellValue));
				//Update cell through inverse model
				l += this->parameters[0] + this->parameters[2];
				//Convert back to probability
				prob = 1 - 1 / (1 + exp(l));
				//Update map
				map.setCellValue(lineIndices[j], prob);
			}
		}
	}
}

//Computes all cells, which are in a direct line between the start position and the
//end position.
//NOTE: This function is not in grid.h, because it cannot bet used for arbitrary dimension
std::vector<int> OccupancyGridBresenham::getCellsInLine(const std::vector<double>& start,
		const std::vector<double>& end, const Grid<double>& m){

	std::vector<int> lineIndices; //Flat indices of cells in line between start and end
	//Get start and end index of cell in flat and multidimensional notation
	int startIndex = m.getCellIndex(start);
	int endIndex = m.getCellIndex(end);
	if(!m.checkIndex(startIndex) || !m.checkIndex(endIndex))
		return lineIndices;
	std::vector<int> startIndices = m.flatToMultiIndex(startIndex);
	std::vector<int> endIndices = m.flatToMultiIndex(endIndex);

	//Start Bresenham line drawing algorithm. Implementation from
	//http://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C.2B.2B
	double x1 = start[0];
	double y1 = start[1];
	double x2 = end[0];
	double y2 = end[1];

	//Check if slope of line is larger or smaller than one
	const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
	//If larger, swap values accordingly
	if(steep)
	{
		std::swap(x1, y1);
		std::swap(startIndices[0], startIndices[1]);
		std::swap(x2, y2);
		std::swap(endIndices[0], endIndices[1]);
	}

	//Swap values if end point is left of start point
	if(x1 > x2)
	{
		std::swap(x1, x2);
		std::swap(startIndices[0], endIndices[0]);
		std::swap(y1, y2);
		std::swap(startIndices[1], endIndices[1]);
	}

	//Compute differences
	const double dx = x2 - x1;
	const double dy = fabs(y2 - y1);

	//Set error and step variable
	double error = dx / 2.0f;
	const int ystep = (y1 < y2) ? 1 : -1;

	//Starting value for y is multidim index of start point
	int y = startIndices[1];
	//End point is multidim index of end point
	const int maxX = endIndices[0];
	//Temp variables for calculated indices
	std::vector<int> indices(2);
	int index;
	for(int x=startIndices[0]; x<maxX; x++)
	{
		//Get multidim indices of next cell
		if(steep)
		{
			indices[0] = y;
			indices[1] = x;
		}
		else
		{
			indices[0] = x;
			indices[1] = y;
		}
		//Transform to flat index
		index = m.multiToFlatIndex(indices);
		//Exclude start and end point
		if(index != endIndex && index != startIndex)
			lineIndices.push_back(index);
		//Advance to next cell
		error -= dy;
		if(error < 0)
		{
			y += ystep;
			error += dx;
		}
	}
	return lineIndices;
}

}
