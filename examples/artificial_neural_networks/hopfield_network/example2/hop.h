//Hop.h      V. Rao, H. Rao
//Single layer Hopfield Network with 4 neurons

#include <stdio.h>
#include <iostream>
#include <math.h>

using namespace std;

class neuron
{
protected:
	int activation;
	friend class network;
public:
	int weightv[4];
	neuron() {};
	neuron(int *j) ;
	int act(int, int*);
};


class network
{
public:
	neuron   nrn[4];
	int output[4];
	int threshld(int) ;
	void activation(int j[4]);
	network(int*,int*,int*,int*);

};
