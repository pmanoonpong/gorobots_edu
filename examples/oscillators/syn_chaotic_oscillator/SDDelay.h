#ifndef SDDELAY_H
#define SDDELAY_H

class SDDelay
{
protected:
	int counter_;
	int bufmax_;
	double *buf_;

public:
	SDDelay(int buffersize = 200);
	~SDDelay();

	void push(double input);
	double get(int delay);
};

#endif /* SDDELAY_H */
