// layerk.cpp		V.Rao, H.Rao
// compile for floating point hardware if available




//#include "layer.cpp"

#include "layerk.h"

using namespace std;
// -----------------------------------------
// 				Kohonen layer                              
//------------------------------------------ 



Kohonen_layer::Kohonen_layer(int i, int o, int init_neigh_size)
{
num_inputs=i;
num_outputs=o;
neighborhood_size=init_neigh_size; 
weights = new float[num_inputs*num_outputs];
outputs = new float[num_outputs]; 
}                                


Kohonen_layer::~Kohonen_layer()
{
delete [] weights;
delete [] outputs;
}

void Kohonen_layer::calc_out()
{
// implement lateral competition
// choose the output with the largest
// value as the winner; neighboring
// outputs participate in next weight
// update. Winner's output is 1 while
// all other outputs are zero

int i,j,k;        
float accumulator=0.0;
float maxval;
winner_index=0;
maxval=-1000000;

for (j=0; j<num_outputs; j++)
	{
	
	for (i=0; i<num_inputs; i++)

		{     
		k=i*num_outputs;
		if (weights[k+j]*weights[k+j] > 1000000.0)
			{
			cout << "weights are blowing up\n";
			cout << "try a smaller learning constant\n";
			cout << "e.g. beta=0.02    aborting...\n";
			exit(1);
			}
		outputs[j]=weights[k+j]*(*(inputs+i));
		accumulator+=outputs[j];
		}
	// no squash function
	outputs[j]=accumulator;
	if (outputs[j] > maxval)
		{
		maxval=outputs[j];
		winner_index=j;
		}
	accumulator=0;
	}                     
	
// set winner output to 1
outputs[winner_index]=1.0;
// now zero out all other outputs
for (j=0; j< winner_index; j++)
	outputs[j]=0;
for (j=num_outputs-1; j>winner_index; j--)
	outputs[j]=0;


}


void Kohonen_layer::randomize_weights()
{
int i, j, k;
const unsigned first_time=1;

const unsigned not_first_time=0;
float discard;
float norm;

discard=randomweight(first_time);

for (i=0; i< num_inputs; i++)
	{
	k=i*num_outputs;
	for (j=0; j< num_outputs; j++)
		{
		weights[k+j]=randomweight(not_first_time);
		}
	}	


// now need to normalize the weight vectors
// to unit length
// a weight vector is the set of weights for
// a given output
            
for (j=0; j< num_outputs; j++)
    {
    norm=0;
	for (i=0; i< num_inputs; i++)
		{
		k=i*num_outputs;
		norm+=weights[k+j]*weights[k+j];
        }
	norm = 1/((float)sqrt((double)norm));

    for (i=0; i< num_inputs; i++)
    	{
    	k=i*num_outputs;
    	weights[k+j]*=norm;
    	}
    }


}


void Kohonen_layer::update_neigh_size(int new_neigh_size)
{
neighborhood_size=new_neigh_size;
}


void Kohonen_layer::update_weights(const float alpha)
{
int i, j, k;
int start_index, stop_index;
// learning law: weight_change =
//		alpha*(input-weight)
//	zero change if input and weight
// vectors are aligned 
// only update those outputs that
// are within a neighborhood's distance
// from the last winner     
start_index = winner_index - 
			neighborhood_size;

			

if (start_index < 0)
	start_index =0;
	
stop_index = winner_index +
			neighborhood_size; 
			

if (stop_index > num_outputs-1)
	stop_index = num_outputs-1;
	


for (i=0; i< num_inputs; i++) 
	{
	k=i*num_outputs;
	for (j=start_index; j<=stop_index; j++)
		weights[k+j] += 
			alpha*((*(inputs+i))-weights[k+j]);
    	}
                                              
}

void Kohonen_layer::list_weights()
{
int i, j, k;

for (i=0; i< num_inputs; i++)
	{
	k=i*num_outputs;
	for (j=0; j< num_outputs; j++)
		cout << "weight["<<i<<","<<
			j<<"] is: "<<weights[k+j];
	}
			
} 

void Kohonen_layer::list_outputs()
{
int i;

for (i=0; i< num_outputs; i++)
	{
	cout << "outputs["<<i<<
			"] is: "<<outputs[i];
	}
			
} 


float Kohonen_layer::get_win_dist()
{
int i, j, k;
j=winner_index;     
float accumulator=0;

       
float * win_dist_vec = new float [num_inputs];

for (i=0; i< num_inputs; i++)
	{
	k=i*num_outputs;
	win_dist_vec[i]=(*(inputs+i))-weights[k+j];
    	accumulator+=win_dist_vec[i]*win_dist_vec[i];
    	}
    
win_distance =(float)sqrt((double)accumulator);  
		
delete []win_dist_vec;

return win_distance;
		
}


Kohonen_network::Kohonen_network()
{

}

Kohonen_network::~Kohonen_network()
{
}

void Kohonen_network::get_layer_info()
{
int i;

//------------------------------------------
//
// 	Get layer sizes for the Kohonen network 
//	
// -----------------------------------------                                            
       

                               
cout << " Enter in the layer sizes separated by spaces.\n";
cout << " A Kohonen network has an input layer \n";
cout << " followed by a Kohonen (output) layer \n";

for (i=0; i<2; i++)
	{
	cin >> layer_size[i];
	}
                               
// ------------------------------------------------------
// size of layers: 
//		input_layer 		layer_size[0]
//		Kohonen_layer		layer_size[1]
//-------------------------------------------------------

							
}

void Kohonen_network::set_up_network(int nsz)
{
int i;

// set up neighborhood size
neighborhood_size = nsz;

//-------------------------------------------------------	
// Construct the layers							
//
//-------------------------------------------------------	 



layer_ptr[0] = new input_layer(0,layer_size[0]);

layer_ptr[1] = 
	new Kohonen_layer(layer_size[0],
			layer_size[1],neighborhood_size);
	


for (i=0;i<2;i++)
	{
	if (layer_ptr[i] == 0)
		{
		cout << "insufficient memory\n";
		cout << "use a smaller architecture\n";
		exit(1);
		}
	}

//-------------------------------------------------------	
// Connect the layers
//
//-------------------------------------------------------	 
// set inputs to previous layer outputs for the Kohonen layer

layer_ptr[1]->inputs = layer_ptr[0]->outputs;


}


void Kohonen_network::randomize_weights()
{

((Kohonen_layer *)layer_ptr[1])
		->randomize_weights();
}


void Kohonen_network::update_neigh_size(int n)
{
((Kohonen_layer *)layer_ptr[1])
		->update_neigh_size(n);
}

void Kohonen_network::update_weights(const float a)
{
((Kohonen_layer *)layer_ptr[1])
		->update_weights(a);
}


void Kohonen_network::list_weights()
{
((Kohonen_layer *)layer_ptr[1])
		->list_weights();
}

void Kohonen_network::list_outputs()
{
((Kohonen_layer *)layer_ptr[1])
		->list_outputs();
}

void Kohonen_network::get_next_vector(FILE * ifile)
{
int i; 
float normlength=0;
int num_inputs=layer_ptr[1]->num_inputs;
float *in = layer_ptr[1]->inputs;
// get a vector and normalize it
for (i=0; i<num_inputs; i++)
	{
	fscanf(ifile,"%f",(in+i));
	normlength += (*(in+i))*(*(in+i));
	}
fscanf(ifile,"\n");                                         
normlength = 1/(float)sqrt((double)normlength);
for (i=0; i< num_inputs; i++)
	{
	(*(in+i)) *= normlength;
	}
	
}


void Kohonen_network::process_next_pattern()
{
	layer_ptr[1]->calc_out();
}

float Kohonen_network::get_win_dist()
{
float retval;
retval=((Kohonen_layer *)layer_ptr[1])
		->get_win_dist();
		
return retval;
}
 
int Kohonen_network::get_win_index()
{
return ((Kohonen_layer *)layer_ptr[1])
		->winner_index;
		
}

