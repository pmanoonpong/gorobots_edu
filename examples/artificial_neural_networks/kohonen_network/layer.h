// layer.h		V.Rao, H. Rao
// header file for the layer class heirarchy and
// the network class

#ifndef _LAYER_H_
#define _LAYER_H_

#define MAX_LAYERS	5
#define MAX_VECTORS	100


#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <time.h>


inline float squash(float input)
// squashing function
// use sigmoid -- can customize to something
// else if desired; can add a bias term too
//
{
if (input < -50)
	return 0.0;
else	if (input > 50)
		return 1.0;
	else return (float)(1/(1+exp(-(double)input)));

}


inline float randomweight(unsigned init)
{
int num;
// random number generator
// will return a floating point
// value between -1 and 1

if (init==1)	// seed the generator
	srand ((unsigned)time(NULL));

num=rand() % 100;

return 2*(float(num/100.00))-1;
}


class network;
class Kohonen_network;

class layer
{               
   
public:

	int num_inputs;
	int num_outputs;
	float *outputs;	// pointer to array of outputs
	float *inputs; 	// pointer to array of inputs, which
					// are outputs of some other layer

	friend class network;
	friend class Kohonen_network; // update for Kohonen model
	
public:
                     
	virtual void calc_out()=0;
};

	

class input_layer: public layer
{

private:


public:
	                        
	input_layer(int, int);
	~input_layer();
	virtual void calc_out();
	
};

class middle_layer;

class output_layer:	public layer
{
public:
	
	float * weights;
	float * output_errors; // array of errors at output
	float * back_errors; // array of errors back-propagated
	float * expected_values;	// to inputs
    //friend network;
    
public:
	                        
	               
	output_layer(int, int);
	~output_layer();    
	virtual void calc_out();
	void calc_error(float &);
	void randomize_weights();
	void update_weights(const float);
	void list_weights();
	void write_weights(int, FILE *);
	void read_weights(int, FILE *);
	void list_errors();
	void list_outputs();
};


	

class middle_layer:	public output_layer
{

private:

public:
    middle_layer(int, int);
    ~middle_layer();
	void calc_error();
};	


class network        

{

private:

layer *layer_ptr[MAX_LAYERS];
    int number_of_layers;
    int layer_size[MAX_LAYERS];
    float *buffer;
    fpos_t position;
    unsigned training;
 
public:
    network();
    ~network();
	void set_training(const unsigned &);
	unsigned get_training_value();
	void get_layer_info();
	void set_up_network();
	void randomize_weights();
	void update_weights(const float);
	void write_weights(FILE *);
	void read_weights(FILE *);     
	void list_weights();
	void write_outputs(FILE *);
	void list_outputs();
	void list_errors();
	void forward_prop();
	void backward_prop(float &);
	int fill_IObuffer(FILE *);
	void set_up_pattern(int);
	
};	    
    
#endif
