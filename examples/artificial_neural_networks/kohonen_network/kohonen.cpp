// kohonen.cpp		V. Rao, H. Rao
// Program to simulate a Kohonen map

//#include "layerk.cpp"
#include "layerk.h"

#define INPUT_FILE "input.dat"
#define OUTPUT_FILE "kohonen.dat"
#define dist_tol	0.05


/*
void main()
{

int neighborhood_size, period;
float avg_dist_per_cycle=0.0;
float dist_last_cycle=0.0;
float avg_dist_per_pattern=100.0; // for the latest cycle
float dist_last_pattern=0.0; 
float total_dist;
float alpha;
unsigned startup;
int max_cycles;
int patterns_per_cycle=0;

int total_cycles, total_patterns;



// create a network object
Kohonen_network knet;


FILE * input_file_ptr, * output_file_ptr;
                                            
// open input file for reading
if ((input_file_ptr=fopen(INPUT_FILE,"r"))==NULL)
		{
		cout << "problem opening input file\n";
		exit(1);
		}

// open writing file for writing
if ((output_file_ptr=fopen(OUTPUT_FILE,"w"))==NULL)
		{
		cout << "problem opening output file\n";
		exit(1);
		}



// -----------------------------------------
//	Read in an initial values for alpha, and the
//  neighborhood size. 
//  Both of these parameters are decreased with
//  time. The number of cycles to execute before
//  decreasing the value of these parameters is
//	called the period. Read in a value for the
//	period.
// -----------------------------------------
	cout << " Please enter initial values for:\n";
	cout << "alpha (0.01-1.0),\n";
	cout << "and the neighborhood size (integer between 0 and 50)\n";
	cout << "separated by spaces, e.g. 0.3 5 \n ";
	
	cin >> alpha >> neighborhood_size ;
	
	cout << "\nNow enter the period, which is the\n";
	cout << "number of cycles after which the values\n";
	cout << "for alpha the neighborhood size are decremented\n";
	cout << "choose an integer between 1 and 500 , e.g. 50 \n";
	
	cin >> period;
	

	// Read in the maximum number of cycles
	// each pass through the input data file is a cycle
	cout << "\nPlease enter the maximum cycles for the simulation\n";
	cout << "A cycle is one pass through the data set.\n";
	cout << "Try a value of 500 to start with\n\n";

	cin >> max_cycles;





// the main loop
//
//      continue looping until the average distance is less than
// 		the tolerance specified at the top of this file
//		, or the maximum number of
// 		cycles is exceeded; 

// initialize counters
total_cycles=0; // a cycle is once through all the input data
total_patterns=0; // a pattern is one entry in the input data



// get layer information
knet.get_layer_info();

// set up the network connections
knet.set_up_network(neighborhood_size);

// initialize the weights

// randomize weights for the Kohonen layer
// note that the randomize function for the
// Kohonen simulator generates
// weights that are normalized to length = 1
knet.randomize_weights();     

// write header to output file
fprintf(output_file_ptr,
	"cycle\tpattern\twin index\tneigh_size\tavg_dist_per_pattern\n");
	
fprintf(output_file_ptr,
	"------------------------------------------------------\n");
		
// main loop

startup=1;
total_dist=0;

while (		
			(avg_dist_per_pattern > dist_tol)
			&& (total_cycles < max_cycles)
		
			|| (startup==1)
			)
{
startup=0;
dist_last_cycle=0; // reset for each cycle
patterns_per_cycle=0;
// process all the vectors in the datafile
  
while (!feof(input_file_ptr))
	{
	knet.get_next_vector(input_file_ptr);

	// now apply it to the Kohonen network
	knet.process_next_pattern();

    dist_last_pattern=knet.get_win_dist();
    
    // print result to output file
    fprintf(output_file_ptr,"%i\t%i\t%i\t\t%i\t\t%f\n",
	total_cycles,total_patterns,knet.get_win_index(),
	neighborhood_size,avg_dist_per_pattern);
			
	total_patterns++;
	
	// gradually reduce the neighborhood size
	// and the gain, alpha
	if (((total_cycles+1) % period) == 0)           
		{
		if (neighborhood_size > 0)
			neighborhood_size --;
		knet.update_neigh_size(neighborhood_size);
		if (alpha>0.1)
			alpha -= (float)0.1;
		}
		

	patterns_per_cycle++;
	dist_last_cycle += dist_last_pattern;
	knet.update_weights(alpha);
	dist_last_pattern = 0;
    }

avg_dist_per_pattern= dist_last_cycle/patterns_per_cycle;
total_dist += dist_last_cycle;
total_cycles++;





fseek(input_file_ptr, 0L, SEEK_SET); // reset the file pointer
				// to the beginning of
				// the file


} // end main loop

cout << "\n\n\n\n\n\n\n\n\n\n\n";
cout << "---------------------------------------------------\n";
cout << " 	done \n";

avg_dist_per_cycle= total_dist/total_cycles;



cout << "\n";
cout << "---->average dist per cycle = " << avg_dist_per_cycle << " <---\n";
cout << "---->dist last cycle = " << dist_last_cycle << " <---\n";
cout << "->dist last cycle per pattern= " << avg_dist_per_pattern << " <---\n";
cout << "------------>total cycles = " << total_cycles << " <---\n";
cout << "------------>total patterns = " << total_patterns << " <---\n";
cout << "---------------------------------------------------\n";
// close the input file
fclose(input_file_ptr);
}
*/
