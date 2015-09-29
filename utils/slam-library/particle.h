#ifndef SB_PARTICLE_H_
#define SB_PARTICLE_H_

//#include "state2D.h"
#include "grid.h"
#include <vector>

namespace SLAM{

//TODO: Maybe extend to a complete class at some point
//Structure to represent one particle in a particle filter. 
struct Particle{
	Particle(){
		this->state.clear();
		this->weight = -1;
		this->map = Grid<double>();
	}
	Particle(std::vector<double> state, double w, Grid<double> map){
		this->state = state;
		this->weight = w;
		this->map = map;
	}
	
	std::vector<double> state; //State of the robot
	double weight; //Weight of the particle
	Grid<double> map; //Map belonging to the particle
};

}

#endif