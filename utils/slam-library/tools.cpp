#include "tools.h"

namespace SLAM{

double Tools::sampleUniform(double low, double high) {
	return low + static_cast <float> (this->varUni()) /( static_cast <float> (this->dist.max()/(high - low)));
}

double Tools::sampleNormal(double variance) {
	//Algorithm after: Thrun, Probabilistic Robotics, table 5.4
	double s = 0;
	for (int i = 1; i < 13; i++)
		s += this->sampleUniform(-sqrt(variance),sqrt(variance));
	return 0.5 * s;
}

double Tools::sampleNormal(double mean, double variance){
	return this->sampleNormal(variance) + mean;
}

int Tools::drawWeights(std::vector<double>& weights, double sum) {
	//Source: http://eli.thegreenplace.net/2010/01/22/weighted-random-generation-in-python/
	double rand = this->sampleUniform(0,1) * sum;
	for(unsigned int i = 0; i < weights.size(); i++){
		rand -= weights[i];
		if(rand < 0)
			return i;
	}
	return -1;
}

int Tools::drawWeights(std::vector<double>& weights) {
	double sum =std::accumulate(weights.begin(),weights.end(),0.0);
	return this->drawWeights(weights,sum);
}

double Tools::probNormal(double value, double variance){
	return 1./sqrt(2 * M_PI * variance) * exp(-0.5 * value * value / variance);
}

double Tools::wrapAngle(double angle){
	if(angle > M_PI)
		return angle - 2*M_PI;
	else if(angle < -M_PI)
		return angle + 2*M_PI;
	else
		return angle;
}

}

