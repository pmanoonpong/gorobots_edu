#include "SDHys.h"
#include "math.h"

/** @brief SDHys
  *
  * @todo: constructor
  */
SDHys::SDHys(double inverse, double up, double down): EPSILON(0.5)
{
	ahys_ = 0;
	setInverse(inverse);
	setSlope(up, down);
	setParams(1.1, 10., 2);
}

/** @brief tanh
  *
  * @todo: calculate tanh output
  */
double SDHys::tanh_(double x)
{
 	return 2./(1.+exp(-mtanh_*x))-1;
// 	return 1./(1.+exp(-10*x))-1;
}


/** @brief getOutput
  *
  * @todo: return current output of hysteresis neuron
  */
double SDHys::getOutput()
{
	return output_;
}

void SDHys::setSlope(double up, double down)
{
	slopeUP_ = up;
	slopeDOWN_ = down;
}

void SDHys::setInverse(double inverse)
{
	inverse_ = inverse;
}

void SDHys::setParams(double w, double m, int update)
{
	whys_ = w;
	mtanh_ = m;
	update_ = update;
}


/** @brief step
  *
  * @todo: calculate next value of hysteresis neuron and return current output
  */
double SDHys::step(double input)
{
	ahysold_ = ahys_;
	ahys_ = tanh_(whys_*ahys_+ input);

	//integrate (linearize) hysteresis signal
	if(fabs(ahys_-ahysold_)>EPSILON)
	{
		if(ahys_ >0)
			output_ = -inverse_;
		else
			output_ = inverse_;
	}

	if(inverse_*ahys_ >0)
	{
		output_ += slopeUP_;
	}
	else
	{
		output_ += slopeDOWN_;
	}

	//set buffer value for delys
	if(output_>1.)
		output_ = 1.;
	else if(output_<-1.)
		output_ = -1.;

	return output_;
}
