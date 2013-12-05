//percept.h        V. Rao, H. Rao
// Perceptron model

#include <stdio.h>
#include <iostream>
#include <math.h>

using namespace std;

class ineuron
{
protected:
	float weight;
	float activation;
	friend class oneuron;
public:
	ineuron() {};
	ineuron(float j) ;
	float act(float x);
};


class oneuron
{
protected:
	int output;
	float activation;
	friend class network;
public:
	oneuron() { };
	void actvtion(float x[4], ineuron *nrn);
	int outvalue(float j) ;
};


class network
{
public:
	ineuron   nrn[4];
	oneuron   onrn;
	network(float,float,float,float);

};
