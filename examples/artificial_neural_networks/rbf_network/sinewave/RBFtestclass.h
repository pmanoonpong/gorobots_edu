


#include <iostream>
#include "ngnet.h"

using namespace std;


class RBFtest {
public:
	NGNet* ngnet;
	Cell* cell;
	/////////params
	int number_of_inputs;
	int number_of_outputs;
	int number_of_hidden_neuron;
	int number_of_data_points;
	double max_limit;
	double min_limit;
	int max_epochs;
	double min_error;
	double learning_rate;
	vector<double> dataset_in;
	vector<double> dataset_out;
	double* wbasis;

	//case 0 --> time series
	//case 1 --> sin wave
	int case_variable;


	RBFtest();
	~RBFtest();
	void test1_SinWave();
	void test2_TimeSeries();
	void initalize_network();
	double train(vector<double> in, vector<double> out, double wbasis[]);
};
