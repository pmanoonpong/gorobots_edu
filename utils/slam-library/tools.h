#ifndef SB_TOOLS_H_
#define SB_TOOLS_H_

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include <boost/random.hpp>
#include <boost/random/uniform_int_distribution.hpp>

namespace SLAM{

//Different support functions used throughout the SLAM project.	
//A class is necessary to be able properly handle the initialization of all stuff related to
//random numbers (seed, distribution, ...)
class Tools{
public:
	//Default constructor. Use current time to seed random generator
	Tools() : rng(), varUni(this->rng, this->dist){};
	//Init random generator with given seed
	Tools(int seed) : rng(seed), varUni(this->rng, this->dist){};

	//Returns a sample of a zero centered Gaussian distribution with given variance
	double sampleNormal(double variance) ;
	//Returns a sample of a Gaussian distribution with given mean and variance
	double sampleNormal(double mean, double variance);
	//Returns a sample from the uniform distribution between low and high
	double sampleUniform(double low, double high);
	//Draw an item according to the given weights
	int drawWeights(std::vector<double>& weights) ;
	//Draw an item according to given weights, but sum of weights is already computed
	int drawWeights(std::vector<double>& weights, double sum) ;
	//Wrap angle from [-3*pi, 3*pi] to [-pi,pi]
	static double wrapAngle(double angle);
	//Returns the probability of value under a zero centered Gaussian distribution with given variance
	static double probNormal(double value, double variance);

private:
	//Stuff from boost/random
	boost::random::mt19937 rng;
	boost::random::uniform_int_distribution<> dist;
	boost::random::variate_generator<boost::random::mt19937&,boost::random::uniform_int_distribution<> > varUni;
};
}

#endif
