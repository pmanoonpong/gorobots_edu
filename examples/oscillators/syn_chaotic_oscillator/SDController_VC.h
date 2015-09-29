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

#include <dinput.h> //JoyStick From DirectX/   in lib= winmm.lib /****

using namespace std;

class SDController
{
	ofstream sFile;
protected:
	
//	double o1_, o2_;
	double activityO1, activityO2;
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


  //Input to switch on-off learning
//	double lr_;
   	
  double w11_,w12_,w21_,w22_, theta1_,theta2_;

    const int PERIODMAX;
	const double FAC;

	//double max;
	int percount;

	// hysteresis variables:
	SDHys lin_[2];
	double thetahys_[2];

	int update_;
	int normalize_;
	
  bool learning_;
	int perc_;	

	double 	sigmoid_(double x);
	void 		oscistep();
	void 		targetinput();

public:

    SDController();
	~SDController();
	void 		step();
	void 		reset();
	void 		setPeriod(int period);
	double 	getOutput1() {return lin_[0].getOutput();};
	double 	getOutput2() {return lin_[1].getOutput();};
	int			getPeriod() {return period_;};

	double o1_, o2_;

	 //Input to switch on-off learning
	double lr_;
};


#endif /* SDCONTROLLER_H */
