
#ifndef __SERIAL_H__
#define __SERIAL_H__

// #################### definitions #####################

#include <math.h>
#include <string.h>
#include <stdio.h>



///////// Save text////////////////////////
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
///////////////////////////////////////////


////Define Input,  Hidden, Output
const int numberHidden = 1;
const int numberInput = 2;
const int numberOutput = 1;

const double BP_LEARNING = 0.95;//0.7

  


class Serial
{
  public:

    Serial();
    ~Serial();

	
	// --- Save text------------//
		ofstream saveFile1;
	//-------------------------//


    void saveData();

	double FeedforwardNetwork(double i0, double i1, double d);
    double Run(double i0, double i1);

	//Select one of this
	bool set_logistic;
	bool set_tanh;
	bool momentum;

	//****Transfer function****///
	double sigmoid(double num);
 	double tanh(double num);
	double step(double num);
    double out_step;

	double deltaOutput;
	double deltaHidden[numberHidden];

    
    double WeightO_I[numberOutput][numberInput]; 
    double DeltaWeightO_I[numberInput];


	double Input[numberInput];
	double WeightH_I[numberHidden][numberInput];
	double DeltaWeightH_I[numberHidden][numberInput];


    double BiasH[numberHidden];
	double WeightH_B[numberHidden];
	double DeltaWeightH_B[numberHidden];



	double a_Hidden[numberHidden];
	double o_Hidden[numberHidden];


    double BiasO[numberOutput];
	double WeightO_B[numberOutput];
    double DeltaWeightO_B[numberOutput];
   
	double WeightO_H [numberOutput][numberHidden];
	double DeltaWeightO_H [numberOutput][numberHidden];

	
	double a_Output[numberOutput];
    double o_Output[numberOutput];

	double error;


};

#endif
