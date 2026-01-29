/**
 * @author Poramate 1.02.2007
 */

#include "Serial.h"
#include "time.h"


// ----------------------------------------------------------------------
// ------------ constructor ---------------------------------------------
// ----------------------------------------------------------------------
Serial::Serial()

{
// ------------Initial Serail -------------------------------------------


saveFile1.open("ReadSensors1.txt",ios::out);
saveFile2.open("ReadSensors2.txt",ios::out);
saveFile3.open("ReadSensors3.txt",ios::out);
saveFile4.open("ReadSensors4.txt",ios::out);
saveFile5.open("ReadSensors5.txt",ios::out);
saveFile6.open("ReadSensors6.txt",ios::out);



//***********************************************//

//------------Chaos control---------------------//
	
//    counter = 0;

    sdcontroller_.lr_=0.05;
	
///////////////////////////////Initial Chaos controller///////////////////////	
//	percount = 0;
    sdcontroller_.setPeriod(PER1); //PER4
///////////////////////////////Initial Chaos controller///////////////////////	


//////////////////////////////////End initial variables/////////////////////////////////////////



}



// ----------------------------------------------------------------------
// ------------ destructor ----------------------------------------------
// ----------------------------------------------------------------------

Serial::~Serial()
{

}

 

double Serial::sigmoid(double num)
{
	return 1./(1.+exp(-num));
}


double Serial::tanh(double num)
{
	return 2./(1.+exp(-2.*num))-1.0;
}


//*************Chaos CPG oscillator net*******************//

void Serial::ChaosCPG()

{

	
///////////SET PERIOD ONLY ONE TIME//////////////////////////////
/*
	if(fabs(pre_outputIR6-I6)>EPS)
{

	   if(I6==0 || I6==1) 
		{
			//Slow wave
			sdcontroller_.lr_=0.05;
			sdcontroller_.setPeriod(PER2); //period9
		         
		}

        else if(I6==3) 
		{
		   	//Tetrapod
			sdcontroller_.lr_=0.05;// 
		    sdcontroller_.setPeriod(PER1);//period5
		}
	
           else if(I6==4)
		{
		//Tripod
		  sdcontroller_.setPeriod(PER3); //period4
		  sdcontroller_.lr_=0.05;//       
		}


 }
*/
/////////////////////////////////////////////////////////////////


    sdcontroller_.step();

	outputChaosCPG1 = sdcontroller_.o1_;//sdcontroller_.getOutput2(); //Output
	outputChaosCPG2 = sdcontroller_.o2_;//sdcontroller_.getOutput1(); //Output

	//Original controlled signals = "sdcontroller_.o1_ , sdcontroller_.o2_"
	//Converted signals with hysteresis and slope = "sdcontroller_.getOutput1() , sdcontroller_.getOutput2()"


	AverageChaosCPG = (outputChaosCPG1+outputChaosCPG2)/2;
    printf("CPG\n");

	
saveFile1 <<outputChaosCPG1<<"  "<<outputChaosCPG2<<" "<<AverageChaosCPG<< "   \n" << flush; //SAVE DATA 
	

}

