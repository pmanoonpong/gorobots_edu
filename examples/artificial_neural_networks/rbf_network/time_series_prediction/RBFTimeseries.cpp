/*
 * RBFTimeseries.cpp
 *
 *  Created on: Feb 23, 2014
 *      Author: bassel
 */

#include "RBFTimeseries.h"
#include <stdlib.h>
#include <math.h>
#ifndef absolute
#define absolute(x) (((x) < 0) ? -(x) : (x))
#endif

//Set parameters (for Students)
#define NUMBER_DATA  101 //number of total data points
#define BPM_ITER	2001   //set it equal to the number of total data

ofstream data_RBF;

RBF_Timeseries::~RBF_Timeseries() {
	data_RBF.close();
}

RBF_Timeseries::RBF_Timeseries() {
	data_RBF.open ("out_RBF.txt");

	//In this example we use radial basis function (RBF) neural network to estimate a time series task.
	//To give the network the ability to predict time series, a time window has been used as an input for the
	//network. This window covers a sequence of inputs (e.g. 10 or 20 time steps) and each input sample is
	//fed to the network's inputs (the network has the same number of inputs as the input samples in the time
	//window).


	//After training the network was able to learn the signal and to produce an acceptable approximation for
	//the output signal.The used time window is 10 input
	//samples. Two centers (Gaussian function centers) are used on each dimension of the RBF input space.

	this->test2_TimeSeries();
}


void RBF_Timeseries::initalize_network() {
	cout<<"the initialization of the RBF network"<<endl;
	ngnet = new NGNet(number_of_inputs, number_of_outputs);
	cell = new Cell(number_of_hidden_neuron, number_of_inputs, number_of_outputs);

	int i;
	int state_index[number_of_inputs];
	double xc[number_of_inputs];

	ngnet->init_incsbox(cell, number_of_inputs, number_of_outputs);
	ngnet->reset_incsbox(cell);

	//***Set it differently according to a number of your inputs, here only 2 inputs
	//basis
	int basis [number_of_inputs];
	double xmax[number_of_inputs], xmin[number_of_inputs];
	double basis_width[number_of_inputs];
	wbasis = new double[number_of_inputs];

	basis[0] = number_of_centers_on_each_dim;

	xmax[0] = max_limit;//max_range1;
	xmin[0] = min_limit;//min_range1;

	//2) find width of each input
	for(i=0; i<number_of_inputs; i++) {
		double var = (xmax[0] - xmin[0])/(2.0*basis[0]); // variance
		basis_width[0] = var;
		double width = 1.0/basis_width[0]; //width of each input
		wbasis[i] = width;
	}

	int nbasis = 0; // increases while adding new hidden neurons
	//a loop should be added for each dimension
	for (state_index[0]=0; state_index[0]<basis[0]; state_index[0]++)
	for (state_index[1]=0; state_index[1]<basis[0]; state_index[1]++)
	for (state_index[2]=0; state_index[2]<basis[0]; state_index[2]++)
	for (state_index[3]=0; state_index[3]<basis[0]; state_index[3]++)
	for (state_index[4]=0; state_index[4]<basis[0]; state_index[4]++)
	for (state_index[5]=0; state_index[5]<basis[0]; state_index[5]++)
	for (state_index[6]=0; state_index[6]<basis[0]; state_index[6]++)
	for (state_index[7]=0; state_index[7]<basis[0]; state_index[7]++)
	for (state_index[8]=0; state_index[8]<basis[0]; state_index[8]++)
	for (state_index[9]=0; state_index[9]<basis[0]; state_index[9]++)
	//for (state_index[1]=0; state_index[1]<basis[1]; state_index[1]++) example for additional dimension
	{
		for (i=0; i<number_of_inputs; i++)
			xc[i]  = (xmin[0]+basis_width[0])+state_index[i]*2.0*basis_width[0];
		ngnet->put_incsbox(cell, number_of_inputs , number_of_outputs, xc, wbasis, &nbasis);
	}

}

void RBF_Timeseries::test2_TimeSeries() {

	number_of_inputs = 10;
	number_of_outputs = 1;
	number_of_centers_on_each_dim = 2;
	number_of_hidden_neuron = pow (number_of_centers_on_each_dim, number_of_inputs);//on each dimension
	max_epochs = 100;
	min_error = 1;
	learning_rate = 0.9;
	max_limit = 1;
	min_limit = -1;

	number_of_data_points = NUMBER_DATA;


	int i_text_loop;
	int ii=0;
	bool initialized = 0;
	double input_temp;
	double target_temp;
	vector<double> m_r0_t;
	vector<double> m_r1_t;

	m_r0_t.resize(NUMBER_DATA);
	m_r1_t.resize(NUMBER_DATA);
	for (int i=1;i<BPM_ITER;i++) {
	//------------Read from the file--------------------------------------------//
	char str[10];
	ifstream b_file ("TestingData_100.txt"); // 2 column data, first column = input, second column = target

	if(!initialized) {
		i_text_loop = 0;
		while(b_file>>str) {//time first column
			input_temp = atof(str);//input
			m_r0_t.at(i_text_loop) = input_temp;

			b_file>> str;
			target_temp = atof(str);//target
			m_r1_t.at(i_text_loop) = target_temp;

			i_text_loop++;
		}
		initialized = true;
	}

	if(ii<i_text_loop)
	  ii++;
	else
	  ii=0;

	dataset_in.push_back(m_r0_t.at(ii));
	dataset_out.push_back(m_r1_t.at(ii));
	//--------------------------------------------------------//
	}

	//for (int i=0; i<dataset_in.size(); i++)
	//	cout<<"i"<<i<<dataset_in[i]<<endl;
	//cout<<"///////////////////////////////////////////"<<endl;
	//for (int i=0; i<dataset_out.size(); i++)
	//		cout<<"i"<<i<<dataset_in[i]<<" ,";
	//int i;
	//cin>>i;

	initalize_network();
	//////////////////////train network
	cout<<"training section"<<endl;
	double error = this->train(dataset_in, dataset_out, wbasis);
}

void RBF_Timeseries::get_inputs(double* in, double* new_in, int index) {
	if (index >= (number_of_inputs-1)) {
		int i1=0;
		int i2=(index-(number_of_inputs-1));
		for (int i=0; i < number_of_inputs; i++) {
			new_in[i1] = in[i2];
			i1++; i2++;
		}
	} else {
		for (int i= 0; i < -(index -(number_of_inputs-1)); i++) {
			new_in[i] = 0;
		}
		int i1=0;
		for (int i= -(index -(number_of_inputs-1)); i < number_of_inputs; i++) {
			new_in[i] = in[i1];
			i1++;
		}
	}
}

double RBF_Timeseries::train(vector<double> in, vector<double> out, double wbasis[]) {
	////calculate errors
	cout<<"start training ............"<<endl;
	cout<<"-----------------------------------------------------------"<<endl;
	double total_error = 100000;
	int num_epochs = 0;
	double u_tmp[number_of_outputs];
	double new_in[number_of_inputs];
	double training_error;
	while ((absolute(total_error) > min_error) && (num_epochs < max_epochs)) { //loop over epochs
		///train section
		training_error = 0;

		for (int i=0; i<((int) (number_of_data_points)); i++) {
			get_inputs(&in[0], new_in, i);
			ngnet->incsbox_output(cell, new_in, u_tmp, &number_of_hidden_neuron);

			double sample_error = out[i] - u_tmp[0];
			training_error += sample_error;
			sample_error = -sample_error;
			ngnet->incsbox_trace(cell, new_in, 0 , &number_of_hidden_neuron);
			ngnet->incsbox_update(cell, new_in, &sample_error, learning_rate, &number_of_hidden_neuron
					, wbasis, 0, 0);
		}
		cout<<"Epoch="<<num_epochs<<", training_error_sqr="<<training_error*training_error<<endl;

		//for (int j=0; j<number_of_inputs; j++)
			//	cout<<"in"<<j<<"="<<new_in[j]<<",";
		//cout<<endl;
		//int i2;
		//cin>>i2;
		num_epochs++;
	}

		cout<<"//////////////////////////////////////////////////////////////"<<endl;
		cout<<"Test the output of the network (assessment section):"<<endl;
		cout<<"-----------------------------------------------------------"<<endl;
		//assessment section
		total_error = 0;
		for (int i=0; i<number_of_data_points; i++) {
			double u_tmp[number_of_outputs];
			double new_in[number_of_inputs];
			get_inputs(&in[0], new_in, i);
			ngnet->incsbox_output(cell, new_in, u_tmp, &number_of_hidden_neuron);
			double current_error = (out[i] - u_tmp[0]);//*(out[i] - u_tmp[0]);
			//total_error += current_error;
			cout<<"t="<<i<<",error (desired - RBF out) ="<<current_error<<",in["<<i<<"]="<<in[i]<<",o="<<out[i]<<endl;
			data_RBF<<u_tmp[0]<<endl;
		}


	return total_error;
}
