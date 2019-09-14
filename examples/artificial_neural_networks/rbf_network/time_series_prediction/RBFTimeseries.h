/*
 * RBFTimeseries.h
 *
 *  Created on: Feb 23, 2014
 *      Author: bassel
 */

#ifndef RBFTIMESERIES_H_
#define RBFTIMESERIES_H_

#include "ngnet.h"

using namespace std;

class RBF_Timeseries {
public:
	NGNet* ngnet;
		Cell* cell;
		/////////params
		int number_of_inputs;
		int number_of_outputs;
		int number_of_hidden_neuron;
		int number_of_data_points;
		int number_of_centers_on_each_dim;
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


		RBF_Timeseries();
		~RBF_Timeseries();
		void test1_SinWave();
		void test2_TimeSeries();
		void initalize_network();
		void get_inputs(double* in, double* new_in, int index);
		double train(vector<double> in, vector<double> out, double wbasis[]);
};

#endif /* RBFTIMESERIES_H_ */
