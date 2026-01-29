
#ifndef __SERIAL_H__
#define __SERIAL_H__

// #################### Include header files #####################

#include <math.h>
#include <string.h>
#include <stdio.h>


///////// header files for save text to debug program/////////////

#include <iostream>
#include <fstream>
#include <string.h>


#include "SDController.h"
#include "SDDelay.h"

using namespace std;

// ######## define COM PORT (Serial connection) ##################




//-----Chaos control parameters----//

const int PER1 = 4;//8;//5;//9;//9;//8;//9**;//Default
/*
const int PER2 = 4;//8;//9;//5;//For rear sensor
const int PER3 = 4;//8;//4;//For (top)back sensor

const int PER4 = 4;//8;//6;//Default //down slope
const int PER5 = 4;//8;//For rear sensor //up slope
const int PER6 = 4;//8;//12;
const int PER7 = 4;//8;//1; 
*/



//---End Chaos control parameters---//


class Serial
{
  public:

    Serial();
    ~Serial();



	void ChaosCPG();
    
	//****Transfer function****///
	double sigmoid(double num);
 	double tanh(double num);


	ofstream outFile; 
	ofstream outFile2;
	ofstream outFile3;

	// --- Save text------------//
		ofstream saveFile1;
		ofstream saveFile2;
		ofstream saveFile3;
		ofstream saveFile4;
		ofstream saveFile5;
		ofstream saveFile6;
	//-------------------------//


	 //---------Chaos control parameters----------------//

  protected:

    SDController sdcontroller_;/////Transfer variable from SDcontroller class
  
	
	//  SDDelay      sddelayH13_, sddelayH14_;
	//	SDDelay      sddelayTL_, sddelayTR_, sddelayCL_, sddelayCR_, sddelayFL_, sddelayFR_;

	//////Chaos CPG/////////
	double outputChaosCPG1; //Output
	double outputChaosCPG2; //Output
	double AverageChaosCPG;

};

#endif // __SERIALMORPHEUS_H__
