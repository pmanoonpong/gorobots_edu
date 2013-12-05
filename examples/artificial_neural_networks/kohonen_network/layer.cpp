// layer.cpp		V.Rao, H.Rao
// compile for floating point hardware if available


#include "layer.h"

using namespace std;


// the next function is needed for Turbo C++
// and Borland C++ to link in the appropriate
// functions for fscanf floating point formats:
static void force_fpf()
{
	float x, *y;
	y=&x;
	x=*y;
}

// -----------------------------------------
// 				input layer                              
//------------------------------------------
input_layer::input_layer(int i, int o)
{                                                       



num_inputs=i;
num_outputs=o;

outputs = new float[num_outputs];
if (outputs==0)
	{
	cout << "not enough memory\n";
	cout << "choose a smaller architecture\n";
	exit(1);
	}
}

input_layer::~input_layer()
{
	delete [] outputs;
}
 

void input_layer::calc_out()
{
//nothing to do, yet
}                    




// -----------------------------------------
// 				output layer                              
//------------------------------------------ 




                      
output_layer::output_layer(int i, int o)
{

num_inputs=i;
num_outputs=o;
weights = new float[num_inputs*num_outputs];
output_errors = new float[num_outputs];
back_errors = new float[num_inputs];
outputs = new float[num_outputs];
expected_values = new float[num_outputs];
if ((weights==0)||(output_errors==0)||(back_errors==0)
	||(outputs==0)||(expected_values==0))
	{
	cout << "not enough memory\n";
	cout << "choose a smaller architecture\n";
	exit(1);
	}

}

output_layer::~output_layer()
{
// some compilers may require the array
// size in the delete statement; those
// conforming to Ansi C++ will not
delete [] weights;
delete [] output_errors;
delete [] back_errors;
delete [] outputs;

}


void output_layer::calc_out()
{                            

int i,j,k;        
float accumulator=0.0;


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
	// use the sigmoid squash function
	outputs[j]=squash(accumulator);
	accumulator=0;
	}

}            



void output_layer::calc_error(float & error)
{        
int i, j, k;                      
float accumulator=0;
float total_error=0;
    
for (j=0; j<num_outputs; j++)
    {
	output_errors[j] = expected_values[j]-outputs[j];
	total_error+=output_errors[j];
	}

error=total_error;

for (i=0; i<num_inputs; i++)
	{
	k=i*num_outputs;
	for (j=0; j<num_outputs; j++)
		{
		back_errors[i]=
			weights[k+j]*output_errors[j];
		accumulator+=back_errors[i];
		}
	back_errors[i]=accumulator;
	accumulator=0;
	// now multiply by derivative of
	// sigmoid squashing function, which is
	// just the input*(1-input)
	back_errors[i]*=(*(inputs+i))*(1-(*(inputs+i)));
	}

}	

void output_layer::randomize_weights()
{
int i, j, k;
const unsigned first_time=1;

const unsigned not_first_time=0;
float discard;

discard=randomweight(first_time);

for (i=0; i< num_inputs; i++)
	{
	k=i*num_outputs;
	for (j=0; j< num_outputs; j++)
		weights[k+j]=randomweight(not_first_time);
	}	
}

void output_layer::update_weights(const float beta)
{
int i, j, k;

// learning law: weight_change =
//		beta*output_error*input

for (i=0; i< num_inputs; i++) 
	{
	k=i*num_outputs;
	for (j=0; j< num_outputs; j++)
		weights[k+j] += 
			beta*output_errors[j]*(*(inputs+i));
    }
                                              
}

void output_layer::list_weights()
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

void output_layer::list_errors()
{
int i, j;

for (i=0; i< num_inputs; i++)
	cout << "backerror["<<i<<
		"] is : "<<back_errors[i]<<"\n";
		
for (j=0; j< num_outputs; j++)
	cout << "outputerrors["<<j<<
			"] is: "<<output_errors[j]<<"\n";
			
}


void output_layer::write_weights(int layer_no,
		FILE * weights_file_ptr)
{   
int i, j, k;

// assume file is already open and ready for
// writing

// prepend the layer_no to all lines of data
// format:
//		layer_no	weight[0,0] weight[0,1] ...
//		layer_no	weight[1,0] weight[1,1] ...
//		...

for (i=0; i< num_inputs; i++)
	{
	fprintf(weights_file_ptr,"%i ",layer_no);
	k=i*num_outputs;
    for (j=0; j< num_outputs; j++)
    	{
    	fprintf(weights_file_ptr,"%f ",
    			weights[k+j]);
    	}                     
    fprintf(weights_file_ptr,"\n");
    }

    
}

void output_layer::read_weights(int layer_no,
		FILE * weights_file_ptr)
{   
int i, j, k;


// assume file is already open and ready for
// reading

// look for the prepended layer_no 
// format:
//		layer_no	weight[0,0] weight[0,1] ...
//		layer_no	weight[1,0] weight[1,1] ...
//		...
while (1)
         
	{
	
	fscanf(weights_file_ptr,"%i",&j);
	if ((j==layer_no)|| (feof(weights_file_ptr)))
		break;
	else      
		{
		while (fgetc(weights_file_ptr) != '\n')
			{;}// get rest of line
		}
	}       

if (!(feof(weights_file_ptr)))	
	{
	// continue getting first line
	i=0;
	for (j=0; j< num_outputs; j++)
			{
		
 		   	fscanf(weights_file_ptr,"%f",
  		  			&weights[j]); // i*num_outputs = 0
    		}                     
   	fscanf(weights_file_ptr,"\n");
    
    
    

	// now get the other lines
	for (i=1; i< num_inputs; i++)
		{
		fscanf(weights_file_ptr,"%i",&layer_no);      
		k=i*num_outputs;
    	for (j=0; j< num_outputs; j++)
    		{
    		fscanf(weights_file_ptr,"%f",
    			&weights[k+j]);
            }

    	}                     
    fscanf(weights_file_ptr,"\n");
    }


else cout << "end of file reached\n"; 

}
void output_layer::list_outputs()
{
int j;

for (j=0; j< num_outputs; j++)
	{
	cout << "outputs["<<j
		<<"] is: "<<outputs[j]<<"\n";
	}
			
}
    

// -----------------------------------------
// 				middle layer                              
//------------------------------------------
 
 
middle_layer::middle_layer(int i, int o):
	output_layer(i,o)
{

}

middle_layer::~middle_layer()
{
delete [] weights;
delete [] output_errors;
delete [] back_errors;
delete [] outputs;
}


void middle_layer::calc_error()
{        
int i, j, k;                      
float accumulator=0;
          
for (i=0; i<num_inputs; i++)
	{
	k=i*num_outputs;
	for (j=0; j<num_outputs; j++)
		{                     
		back_errors[i]=
			weights[k+j]*(*(output_errors+j));
		accumulator+=back_errors[i];
		}                          
	back_errors[i]=accumulator;
	accumulator=0;
	// now multiply by derivative of
	// sigmoid squashing function, which is
	// just the input*(1-input)
	back_errors[i]*=(*(inputs+i))*(1-(*(inputs+i)));
	}

}	

network::network()
{
	//position= 0L;
}

network::~network()
{
int i,j,k;
i=layer_ptr[0]->num_outputs;// inputs
j=layer_ptr[number_of_layers-1]->num_outputs; //outputs
k=MAX_VECTORS;


delete []buffer;
}

void network::set_training(const unsigned & value)
{
training=value;
}

unsigned network::get_training_value()
{
return training;
}


void network::get_layer_info()
{
int i;

//------------------------------------------
//
// 	Get layer sizes for the network 
//	
// -----------------------------------------                                            
       

                               
cout << " Please enter in the number of layers for your network.\n";
cout << " You can have a minimum of 3 to a maximum of 5. \n";
cout << " 3 implies 1 hidden layer; 5 implies 3 hidden layers : \n\n";

cin >> number_of_layers;

cout << " Enter in the layer sizes separated by spaces.\n";
cout << " For a network with 3 neurons in the input layer,\n";
cout << " 2 neurons in a hidden layer, and 4 neurons in the\n";
cout << " output layer, you would enter: 3 2 4 .\n";
cout << " You can have up to 3 hidden layers,for five maximum entries :\n\n";

for (i=0; i<number_of_layers; i++)
	{
	cin >> layer_size[i];
	}
                               
// ------------------------------------------------------
// size of layers: 
//		input_layer 		layer_size[0]
//		output_layer		layer_size[number_of_layers-1]
//		middle_layers		layer_size[1]
//					optional: layer_size[number_of_layers-3]
//					optional: layer_size[number_of_layers-2]
//-------------------------------------------------------

							
}

void network::set_up_network()
{
int i,j,k;
//-------------------------------------------------------	
// Construct the layers							
//
//-------------------------------------------------------	 



layer_ptr[0] = new input_layer(0,layer_size[0]);

for (i=0;i<(number_of_layers-1);i++)
	{
	layer_ptr[i+1] = 
	new middle_layer(layer_size[i],layer_size[i+1]);
	}

layer_ptr[number_of_layers-1] = new 
output_layer(layer_size[number_of_layers-2],layer_size[number_of_layers-1]);

for (i=0;i<(number_of_layers-1);i++)
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
// set inputs to previous layer outputs for all layers,
//	except the input layer

for (i=1; i< number_of_layers; i++)
	layer_ptr[i]->inputs = layer_ptr[i-1]->outputs;


	
// for back_propagation, set output_errors to next layer
//		back_errors for all layers except the output
//		layer and input layer  


for (i=1; i< number_of_layers -1; i++)
	((output_layer *)layer_ptr[i])->output_errors = 
		((output_layer *)layer_ptr[i+1])->back_errors;

// define the IObuffer that caches data from
// the datafile
i=layer_ptr[0]->num_outputs;// inputs
j=layer_ptr[number_of_layers-1]->num_outputs; //outputs
k=MAX_VECTORS;

buffer=new
	float[(i+j)*k];
if (buffer==0)
	{
	cout << "insufficient memory for buffer\n";
	exit(1);
	}
}

void network::randomize_weights()
{ 
int i;

for (i=1; i<number_of_layers; i++)
	((output_layer *)layer_ptr[i])
		->randomize_weights();
}


void network::update_weights(const float beta)
{
int i;

for (i=1; i<number_of_layers; i++)
	((output_layer *)layer_ptr[i])
		->update_weights(beta);
}


void network::write_weights(FILE * weights_file_ptr)
{
int i;

for (i=1; i<number_of_layers; i++)
	((output_layer *)layer_ptr[i])
		->write_weights(i,weights_file_ptr);
}


void network::read_weights(FILE * weights_file_ptr)
{
int i;

for (i=1; i<number_of_layers; i++)
	((output_layer *)layer_ptr[i])
		->read_weights(i,weights_file_ptr);
}


void network::list_weights()
{
int i;

for (i=1; i<number_of_layers; i++)
	{
	cout << "layer number : " <<i<< "\n";
	((output_layer *)layer_ptr[i])
		->list_weights();          
	}
}

void network::list_outputs()
{
int i;

for (i=1; i<number_of_layers; i++)
	{
	cout << "layer number : " <<i<< "\n";
	((output_layer *)layer_ptr[i])
		->list_outputs();          
	}
}
                                
void network::write_outputs(FILE *outfile)
{
int i, ins, outs; 
ins=layer_ptr[0]->num_outputs;
outs=layer_ptr[number_of_layers-1]->num_outputs;
float temp;

fprintf(outfile,"for input vector:\n");

for (i=0; i<ins; i++)
	{
	temp=layer_ptr[0]->outputs[i];
	fprintf(outfile,"%f  ",temp);
	}


fprintf(outfile,"\noutput vector is:\n");

for (i=0; i<outs; i++)
	{           
	temp=layer_ptr[number_of_layers-1]->
	outputs[i];
	fprintf(outfile,"%f  ",temp);
      
	}

if (training==1)
{
fprintf(outfile,"\nexpected output vector is:\n");

for (i=0; i<outs; i++)
	{           
	temp=((output_layer *)(layer_ptr[number_of_layers-1]))->
	expected_values[i];
	fprintf(outfile,"%f  ",temp);
      
	}
}

fprintf(outfile,"\n----------------------\n");

}                              



void network::list_errors()
{
int i;

for (i=1; i<number_of_layers; i++)
	{
	cout << "layer number : " <<i<< "\n";
	((output_layer *)layer_ptr[i])
		->list_errors();          
	}
}



int network::fill_IObuffer(FILE * inputfile)
{
// this routine fills memory with
// an array of input, output vectors
// up to a maximum capacity of 
// MAX_INPUT_VECTORS_IN_ARRAY
// the return value is the number of read
// vectors


int i, k, count, veclength;
      
int ins, outs;

ins=layer_ptr[0]->num_outputs;

outs=layer_ptr[number_of_layers-1]->num_outputs;

if (training==1)
	veclength=ins+outs;
else	
	veclength=ins;

count=0;
while  ((count<MAX_VECTORS)&&
		(!feof(inputfile)))	
	{
	k=count*(veclength);
	for (i=0; i<veclength; i++)
		{
		fscanf(inputfile,"%f",&buffer[k+i]);
		}
	fscanf(inputfile,"\n");
	count++;
	}	
	
if (!(ferror(inputfile)))
	return count;
else return -1; // error condition

}


void network::set_up_pattern(int buffer_index)
{
// read one vector into the network
int i, k;
int ins, outs;

ins=layer_ptr[0]->num_outputs;
outs=layer_ptr[number_of_layers-1]->num_outputs;
if (training==1)
	k=buffer_index*(ins+outs);
else 
	k=buffer_index*ins;

for (i=0; i<ins; i++)
	layer_ptr[0]->outputs[i]=buffer[k+i];

if (training==1)
{
	for (i=0; i<outs; i++)

		((output_layer *)layer_ptr[number_of_layers-1])->
			expected_values[i]=buffer[k+i+ins];
}	

}		
 
 
 
void network::forward_prop()
{
int i;
for (i=0; i<number_of_layers; i++)
	{
	layer_ptr[i]->calc_out(); //polymorphic
				// function
	}
}

void network::backward_prop(float & toterror)
{
int i;

// error for the output layer
((output_layer*)layer_ptr[number_of_layers-1])->
			calc_error(toterror);
	
// error for the middle layer(s)
for (i=number_of_layers-2; i>0; i--)
	{
	((middle_layer*)layer_ptr[i])->
			calc_error();

	}
	
}

