#ifndef SDCONTROLLER_H
#define SDCONTROLLER_H

#include "SDHys.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifndef __unix__
#include <windows.h>
#endif

///////// header files for save text to debug program/////////////

#include <iostream>
#include <fstream>
#include <string.h>

using namespace std;

class SDController
{
	ofstream sFile;
protected:
	
	/****Oscillator Master**********/

	double input1_;
	double input2_;
	int n_;
	int period_;

	double diff_n1_, diff_n2_, diffi_;

	typedef struct
	{
  	double o1, o2;
	} data_t;

   //real *o1_, *o2_;
   data_t* data_;


	double cL_;
	int* DIRS_;
	int NDIRS_;

	//double max;
	int percount;

	int update_;
	int normalize_;

	void 		oscistep();
	void 		targetinput();

	//Post processing
	// hysteresis variables:
	SDHys lin_[2];

	double thetahys_[2];//**********

	double 	input_hys_0;
	double	input_hys_1;
	bool learning_;


/****Oscillator Cli0*************/

	double input1_cli;
	double input2_cli;
	int n_cli;
	int period_cli;

	double diff_n1_cli, diff_n2_cli, diffi_cli;

	typedef struct
	{
  	double o1cli, o2cli;
	} data_tcli;

    //real *o1_, *o2_;
    data_tcli* data_cli;

	double cL_cli;
	int* DIRS_cli;
	int NDIRS_cli;

	//double max;
	int percount_cli;

	int update_cli;
	int normalize_cli;
	
	void 		oscistep_cli();
	void 		targetinput_cli();

	
	//Post processing
	// hysteresis variables:
	SDHys lin_cli[2];

	double thetahys_cli[2];

	double 	input_hys_0_cli;
	double	input_hys_1_cli;

	bool learning_cli;
	
	/****Oscillator Cli1*************/

	double input1_cli1;
	double input2_cli1;
	int n_cli1;
	int period_cli1;
	
	double diff_n1_cli1, diff_n2_cli1, diffi_cli1;

	typedef struct
	{
  	double o1cli1, o2cli1;
	} data_tcli1;

    //real *o1_, *o2_;
    data_tcli1* data_cli1;

	double cL_cli1;
	int* DIRS_cli1;
	int NDIRS_cli1;

	//double max;
	int percount_cli1;

	int update_cli1;
	int normalize_cli1;
	
	void 		oscistep_cli1();
	void 		targetinput_cli1();


	//Post processing
	// hysteresis variables:
	SDHys lin_cli1[2];

	double thetahys_cli1[2];

	double 	input_hys_0_cli1;
	double	input_hys_1_cli1;

	bool learning_cli1;
  	/****Global para**********/


	double w11_,w12_,w21_,w22_, theta1_,theta2_;

	const int PERIODMAX;
	const double FAC;
	int perc_;	
	double 	sigmoid_(double x);
	double 	sigmoid_steep(double x);
	double 	deri_sigmoid_steep(double x);

 
public:

    SDController();
	~SDController();
	
	/****Oscillator Master**********/
	void 		step();	
	void 		setPeriod(int period);
	void 		reset();
	double 	getOutput1() {return lin_[0].getOutput();};
	double 	getOutput2() {return lin_[1].getOutput();};
	int			getPeriod() {return period_;};
	
    //Input to switch on-off learning
	double lr_; 
	double o1_, o2_;
	double activityO1, activityO2;
	
	/****Oscillator Client0*********/
	void 		stepcli();
	void 		setPeriod_cli(int periodcli);
	void 		reset_cli();
	double 	getOutput1cli() {return lin_cli[0].getOutput();};
	double 	getOutput2cli() {return lin_cli[1].getOutput();};
	int			getPeriodcli() {return period_cli;};
		
     //Input to switch on-off learning
	double lr_cli;
	double o1_cli, o2_cli;
	double activityO1_cli, activityO2_cli;
	
	/****Oscillator Client1**********/
	void 		stepcli1();	
	void 		setPeriod_cli1(int periodcli1);
	void 		reset_cli1();
	double 	getOutput1cli1() {return lin_cli1[0].getOutput();};
	double 	getOutput2cli1() {return lin_cli1[1].getOutput();};
	int			getPeriodcli1() {return period_cli1;};

     //Input to switch on-off learning
	double lr_cli1;
	double o1_cli1, o2_cli1;
	double activityO1_cli1, activityO2_cli1;




	///Syn memory//////////////////////
	double  beta_steep;
	double  theta_steep; 

	
	//Syn
	double alpha_syn;
	double q_cli;

	//memory
	int memory;
	int global_counter;
	int golbal_clock;
	int mem_flag; // To reset the memory


};


#endif /* SDCONTROLLER_H */
