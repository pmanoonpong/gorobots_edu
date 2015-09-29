#ifndef SB_GRID_H_
#define SB_GRID_H_

#include "tools.h"
#include <cmath>
#include <vector>
#include <fstream>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace SLAM{

template<class T>
class Grid;

//Class representing one cell (of arbitrary dimension) in a multidimensional grid
template<class T>
class Cell{
	//Grid class should be able to access protected members directly
	friend class Grid<T>;

public:
	T value; //Value of the cell

	//Standard constructor
	//bottomLeft: Location of bottom left corner of cell
	//size: Dimensions of the cell
	//value: Value of the cell
	Cell(std::vector<double> bottomLeft, std::vector<double> size, T value){
		this->dim = bottomLeft.size();
		this->location = bottomLeft;
		this->size = size;
		this->center.resize(dim);
		//Compute center of cell
		for(int i = 0; i < this->dim; i++)
			this->center[i] = this->location[i] + this->size[i]/2.;
		this->value = value;
	}
	//Default constructor
	Cell(){
		//Init with error values
		this->location.resize(0);
		this->center.resize(0);
		this->dim = -1;
	}

	std::vector<double> getCenter() const{
		return this->center;
	}

	std::vector<double> getLocation() const{
		return this->location;
	}

	//Returns the size of the cell in the direction given by index
	double getSize(int index) const {
		return this->size[index];
	}

	//Compute distance of an arbitrary point to center of cell (euclidean distance)
	double getDistanceToCenter(std::vector<double> point) const{
		double dist = 0;
		for(int i = 0; i < this->dim; i++)
			dist += (this->center[i] - point[i]) * (this->center[i] - point[i]);
		return sqrt(dist);
	}

protected:
	int dim; //Number of dimensions of the cell
	std::vector<double> location; //Bottom left corner of cell 
	std::vector<double> center;   //Center of cell
	//Size of cell in each dimension
	std::vector<double> size; 
};


//Class modelling grid representations of arbitrary dimensionality. 
template<class T> //T is type of cell value
class Grid{	
public:
	//Standard constructor
	//start: Bottom left corner of grid
	//end: top right corner of grid
	//resolution: size of cell in each dimension
	//startValue: Value with which cells are initialized.
	Grid(std::vector<double> start, std::vector<double>  end, std::vector<double>  resolution, T startValue){
		this->start = start;
		this->end = end;
		this->resolution = resolution;
		//Compute number of dimensions
		this->dim = start.size();
		//Compute number of cells for each dimension
		this->numberCells.resize(this->dim);
		this->totalNumberCells = 1;
		for(int i = 0; i < this->dim; i++){
			//Distance between start and end point divided by the size of one cell
			this->numberCells[i] = ceil((this->end[i] - this->start[i]) / this->resolution[i]);
			this->totalNumberCells *= this->numberCells[i];
		}

		//Init cells
		this->cells.resize(this->totalNumberCells);
		std::vector<double> position(this->dim);
		std::vector<int> indices;
		for(int i = 0; i < this->totalNumberCells; i++){
			//Convert 1D index to multidimensional index
			indices = this->flatToMultiIndex(i);
			for(int j = 0; j < this->dim; j++)
				//Use multidimensional index to compute 'real world' position of cell
				position[j] = indices[j] * this->resolution[j] + this->start[j];

			//Create new cell
			this->cells[i] = Cell<T>(position, this->resolution, startValue);

		}
	}

	//Read grid from file. Note that file should be written with writeToFile method to ensure correct structure of the data
	//Let dim denote the dimension of the grid. Then the structure should be as follows:
	//First column: Index of cell
	//Second to dim + 1 column: Bottom left corner of each cell
	//dim + 2 to 2*dim + 1: Size of cell for corresponding dimensions
	//Last column: Value of cell
	//
	//Note that cells should be listed, such that the first cell is the bottom left cell, then going "right" in each dimension.
	//All cells should have the same size
	Grid(std::string filename){
		std::ifstream myfile(filename.c_str());
		std::vector<std::string> strs;
		std::vector<double> pos;
		T value;
		std::string line;
		bool firstLine = true;
		this->cells.clear();

		while(getline(myfile,line)){	
			if(line[0]=='#')
				continue;
			//Split line
			split(strs,line, boost::is_any_of("\t"));
			//Check if first line to read
			if(firstLine){
				//Calculate dimension of grid based on assumptions about structure of file
				this->dim = (strs.size() - 2) / 2;
				//Resize vectors
				this->resolution.resize(this->dim);
				this->numberCells.resize(this->dim);
				pos.resize(this->dim);
				//Read resolution
				for(int i = 0; i < this->dim; i++)
					this->resolution[i] = boost::lexical_cast<double>(strs[i + this->dim + 1]);
				firstLine = false;
			}
			//Read position of bottom left corner of cell
			for(int i = 0; i < this->dim; i++)
				pos[i] = boost::lexical_cast<double>(strs[1 + i]);
			//Read value of cell
			value = boost::lexical_cast<T>(strs[2*this->dim + 1]);
			this->cells.push_back(Cell<T>(pos,this->resolution,value));
		}
		//Read start and end point of grid
		this->start = this->cells[0].location;
		this->end = this->cells[this->cells.size() - 1].location;
		//Compute number of cells for each dimension
		for(int i = 0; i < this->dim; i++){
			//End point is TOP RIGHT corner not bottom left
			this->end[i] += this->resolution[i];
			this->numberCells[i] = ceil((this->end[i] - this->start[i]) / this->resolution[i]);
		}
		this->totalNumberCells = this->cells.size();
	}

	//Default constructor. Inits an empty grid.
	Grid(){
		this->dim = -1;
		this->totalNumberCells = -1;
		this->cells.clear();
	}

	//Returns the index of the cell, in which the given point is lying
	//This function may return invalid indices. If this is of interest, the calling
	//function should check, if index is out of bounds (e.g. via checkIndex() )
	int getCellIndex(std::vector<double>  location) const{
		//Compute distance from location to bottom left corner in real world coordinates
		std::vector<int> indices(this->dim);
		for(int i = 0; i < this->dim; i++)
			indices[i] = floor( (location[i] - this->start[i]) / this->resolution[i] );
		//Convert to 1D index
		return this->multiToFlatIndex(indices);
	}

	//Access value of cell specified by location
	//If location is not in grid, false is returned and value will not be overwritten
	bool getCellValue(std::vector<double> location, T& value) const{
		int index = this->getCellIndex(location);
		return this->getCellValue(index, value);
	}

	//Access value of cell specified by index
	//If index is out of bounds false is returned and value will not be overwritten
	bool getCellValue (int index, T& value) const{
		bool indexCorrect = this->checkIndex(index);
		if(indexCorrect)
			value = this->cells[index].value;
		return indexCorrect;
	}

	//Sets the given value to the cell specified by its location
	//Returns if index specifies a cell is within the grid
	bool setCellValue(std::vector<double> location, T value){
		int index = this->getCellIndex(location);
		return this->setCellValue(index, value);

	}

	//Sets the given value to the cell specified by its index.
	//Returns if index specifies a cell is within the grid
	bool setCellValue(int index, T value){
		bool indexCorrect = this->checkIndex(index);
		if(indexCorrect)
			this->cells[index].value = value;
		return indexCorrect;
	}

	//Set all cells to a given value
	void setAllCells(T value){
		for(int i = 0; i < this->totalNumberCells; i++)
			this->cells[i].value = value;
	}

	//Writes map to a file, e.g. to be able to plot it. 
	//Grids written with this method may be read via the corresponding constructor later on
	void writeToFile (std::string fileName) const{
		//NOTE: If this method is changed, make sure to adapt the corresponding constructor
		std::ofstream file;
		file.open(fileName.c_str());
		if(file.is_open()){
			file <<"#Index\tposX\tposY\tsizeX\tsizeY\toccupied"<<std::endl;
			for(unsigned int i = 0; i < this->cells.size(); i++){
				file << i <<"\t"; //First write index of cell
				for(int j = 0; j < this->dim; j++)
					file << this->cells[i].location[j]<<"\t"; //Then location of cell
				for(int j = 0; j < this->dim; j++)
					file << this->cells[i].getSize(j)<<"\t"; //Then size of cell
				file << this->cells[i].value << std::endl; //Last: Value of cell
			}
		}
		file.close();
	}

	//Check if given index is inside the grid or not
	//Returns true if index is valid
	bool checkIndex(int index) const{
		return !(index < 0 || index >= this->cells.size());
	}

	//Get total number of cells
	int getNumberCells() const{
		return this->totalNumberCells;
	}

	//Get cell specified by its index
	Cell<T> getCell(int index) const{
		return this->cells[index];
	}

	//Get coordinates of bottom left corner
	std::vector<double> getStart(){
		return this->start;
	}

	//Get coordinates of top right corner
	std::vector<double> getEnd(){
		return this->end;
	}

	//Convert 1D index to multidimensional index
	std::vector<int> flatToMultiIndex(int index) const{
		std::vector<int> indices(this->dim);
		//Compute product number of elements from first dimension to next to last 
		int product = 1;
		for(int i = 0; i < this->dim-1; i++)
			product *= this->numberCells[i];

		//Compute multidimensional indices backwards (start with highest dimension)
		int currentDimension= this->dim - 1;
		//the basic idea is to remove the influence of the higher dimensions and
		//evaluate the modified index
		int reducedIndex = index;

		//Loop through all indices
		while(currentDimension > 1){
			//Compute index in current dimension by using integer division
			indices[currentDimension] = reducedIndex / product;
			//Remove influence of current dimension by subtracting the corresponding amount of indices
			reducedIndex -= (reducedIndex / product) * product;
			//Update current dimension
			currentDimension--;
			//Update product variable (to hold only product up to current dimension -1 )
			product /= this->numberCells[currentDimension];
		}
		//2D case is clear and can be filled out statically
		indices[1] = reducedIndex / this->numberCells[0];
		indices[0] = reducedIndex % this->numberCells[0];
		return indices;
	}

	//Convert multidimensional index to 1D index
	//Note that index is not verified by this method. Use checkIndex to do this
	int multiToFlatIndex(std::vector<int> indices) const{
		int index = indices[0];
		int product = 1;
		//Check index of first dimension manually, as loop starts at i=1
		if(indices[0] < 0 || indices[0] >= this->numberCells[0]){
			return -1;
		}
		//Sum up contribution of each dimension by multiplying elements in this dimension with index in this dimension
		for(int i = 1; i < this->dim; i++){
			//Check if index is out of bounds
			if(indices[i] < 0 || indices[i] >= this->numberCells[i]){
				index = -1;
				break;
			}
			product *= this->numberCells[i-1];
			index += product * indices[i];

		}
		return index;
	}

private:
	int dim; //Number of dimensions of grid
	std::vector<double>  start; //Bottom left corner of the map
	std::vector<double> end; //Top right corner of the map
	std::vector<double> resolution; //Size of one cell in each dimension
	std::vector<int> numberCells; //Number of cells in each dimensions
	int totalNumberCells; //total number of cells
	std::vector<Cell<T> > cells; //All cells contained in map


};
}

#endif
