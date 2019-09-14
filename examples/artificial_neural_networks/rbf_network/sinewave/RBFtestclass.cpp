/*
 * RBFtestclass.cpp
 *
 *  Created on: Dec 6, 2013
 *      Author: bassel
 */



#include "RBFtestclass.h"
#include <stdlib.h>
#include <math.h>
#ifndef absolute
#define absolute(x) (((x) < 0) ? -(x) : (x))
#endif



//Set parameters (for Students)
#define NUMBER_DATA  2001 //number of total data
#define BPM_ITER	2001   //set it equal to the number of total data



void RBFtest::initalize_network() {
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

	basis[0] = number_of_hidden_neuron;

	xmax[0] = max_limit;//max_range1;
	xmin[0] = min_limit;//min_range1;

	//2) find width of each input
	for(i=0; i<number_of_inputs; i++) {
		double var = (xmax[i] - xmin[i])/(2.0*basis[i]); // variance
		basis_width[i] = var;
		double width = 1.0/basis_width[i]; //width of each input
		wbasis[i] = width;
	}


	int nbasis = 0; // increases while adding new hidden neurons


	//a loop should be added for each dimension
	for (state_index[0]=0; state_index[0]<basis[0]; state_index[0]++)
	//for (state_index[1]=0; state_index[1]<basis[1]; state_index[1]++) example for additional dimension
	{
		for (i=0; i<number_of_inputs; i++)
			xc[i]  = (xmin[i]+basis_width[i])+state_index[i]*2.0*basis_width[i];
		ngnet->put_incsbox(cell, number_of_inputs , number_of_outputs, xc, wbasis, &nbasis);
	}
}

//Transform from a random signal to a sine wave signal
//Using 1 input and 1 output and 10 hidden neurons
void RBFtest::test1_SinWave() {
	number_of_inputs = 1;
	number_of_outputs = 1;
	number_of_hidden_neuron = 10;
	number_of_data_points = 150;
	max_epochs = 2000;
	min_error = 0.001;
	learning_rate = 0.005;
	max_limit = 3.14;
	min_limit = -3.14;



	//generate data from sin function
	cout<<"Generate training data - 80% training points, 20% testing points"<<endl;
	//double dataset [number_of_data_points][2];
	for (int i=0; i<number_of_data_points; i++) {
		double in = (((double)rand()/RAND_MAX)*6) - 3; //-3 --> +3
		double out = sin(in); //-3 --> +3
		dataset_in.push_back(in);
		dataset_out.push_back(out);
	}

	initalize_network();
	//////////////////////train network
	cout<<"training section"<<endl;
	double error = this->train(dataset_in, dataset_out, wbasis);

		///////////////////////////////////print examples
	cout<<"///////////////////////////////////////////////////////"<<endl;
	cout<<"Small test to see the efficiency of the estimation ... (estimation, target output compression)"<<endl;
	cout<<"-------------------------------------------------------"<<endl;
	for (int i=0; i<100; i++) {
		double in = (((double)rand()/RAND_MAX)*6) - 3; //-3 --> +3
		double out = sin(in); //-3 --> +3
		double outputnet = 0;
		ngnet->incsbox_output(cell, &in, &outputnet, &number_of_hidden_neuron);
		cout<<"in = "<<in<<", sin(in) ="<<out<<", RBF out = "<<outputnet<<endl;

	}
}

// Using motor signal = input and foot contact signal = output
void RBFtest::test2_TimeSeries() {

	number_of_inputs = 1;
	number_of_outputs = 1;
	number_of_hidden_neuron = 30;
	max_epochs = 20000;
	min_error = 0.001;
	learning_rate = 0.5;
	max_limit = 1;
	min_limit = -1;

	number_of_data_points = NUMBER_DATA;


	int i_text_loop;
	int ii = 0;
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
	ifstream b_file ("TestingData_2000.txt"); // 2 column data, first column = input, second column = target

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

	initalize_network();
	//////////////////////train network
	cout<<"training section"<<endl;
	double error = this->train(dataset_in, dataset_out, wbasis);
}

RBFtest::RBFtest() {


	//case 0 --> time series
	//case 1 --> sin wave



	////time series doesn't work with RBF structure therefore you will notice that the final error
	//// is big and i doesn't drop to a small number
	////the solution for time series cases is recurrent neural network and this example provide a proof
	////that RBF network is not able to work with time series


	/////case 1: the estimation of the sin wave works perfectly using RBF network ;)

	cout<<"please enter the test case number:"<<endl;
	cout<<"0 ---> time series case"<<endl;
	cout<<"1 ---> sin wave"<<endl;
	int casenumber;
	cin>>casenumber;
	case_variable = casenumber;


	if (case_variable == 1)
		this->test1_SinWave();
	else
		this->test2_TimeSeries();
}



double RBFtest::train(vector<double> in, vector<double> out, double wbasis[]) {
////calculate errors

	cout<<"start training ............"<<endl;
	cout<<"-----------------------------------------------------------"<<endl;
	double total_error = 100000;
	int num_epochs = 0;
	while ((absolute(total_error) > min_error) && (num_epochs < max_epochs)) { //loop over epochs
		///train section
		for (int i=0; i<((int) (number_of_data_points*0.8)); i++) {
			double u_tmp[number_of_outputs];
			ngnet->incsbox_output(cell, &in[i], u_tmp, &number_of_hidden_neuron);
			double sample_error = out[i] - u_tmp[0];
			sample_error = -sample_error;
			ngnet->incsbox_trace(cell, &in[i], 0 , &number_of_hidden_neuron);
			ngnet->incsbox_update(cell, &in[i], &sample_error, learning_rate, &number_of_hidden_neuron
					, wbasis, 0, 0);
		}

		//assessment section
		total_error = 0;
		for (int i=((int) (number_of_data_points*0.8)) + 1; i<number_of_data_points; i++) {
			double u_tmp[number_of_outputs];
			ngnet->incsbox_output(cell, &in[i], u_tmp, &number_of_hidden_neuron);

			total_error += (out[i] - u_tmp[0])*(out[i] - u_tmp[0]);
		}
		cout<<"epoch ("<<num_epochs + 1<<"): training error = "<<total_error<<endl;
		num_epochs++;
	}
	return total_error;
}

RBFtest::~RBFtest(){

}

