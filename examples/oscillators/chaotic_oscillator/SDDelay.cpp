#include "SDDelay.h"


SDDelay::SDDelay(int buffersize)
{
	bufmax_ = buffersize;
	buf_ = new double[bufmax_];
	for(int i=0; i<bufmax_; ++i)
		buf_[i] = 0;
	counter_ = 0;
}

SDDelay::~SDDelay()
{
	delete[] buf_;
}

void SDDelay::push(double input)
{
	if(++counter_>=bufmax_)
		counter_ = 0;
	buf_[counter_] = input;
}

double SDDelay::get(int delay)
{
	int index = counter_-delay;
	return buf_[(index<0)?(bufmax_+index):index];
}
