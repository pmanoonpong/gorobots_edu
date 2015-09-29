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
    sdcontroller_.lr_cli=0.05;	
    sdcontroller_.lr_cli1=0.05;

///////////////////////////////Initial Chaos controller///////////////////////	
//	percount = 0;
	sdcontroller_.setPeriod(5); //PER4 Master
	sdcontroller_.setPeriod_cli(2); //PER4 cli
	sdcontroller_.setPeriod_cli1(4); //PER4 cli1
///////////////////////////////Initial Chaos controller///////////////////////	

	counter_Switching = 0;

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
	// Start with Period 4

	counter_Switching++;
	sdcontroller_.memory++;
	sdcontroller_.theta_steep = 8000;//Through learning

////Change period of MASTER
//Syn
	if(counter_Switching == 1)
    {	
		//sdcontroller_.memory = 100000;

		sdcontroller_.lr_=0.05;
		sdcontroller_.setPeriod(8); //period6

		//sdcontroller_.lr_cli=0.05;	
		//sdcontroller_.setPeriod_cli(5); //PER4 cli
		
		sdcontroller_.lr_cli1=0.05;
		sdcontroller_.setPeriod_cli1(5); //PER4 cli1

	}
//Not Syn
	if(counter_Switching == 10000)
    {	

        sdcontroller_.memory = 0;

		sdcontroller_.lr_=0.05;
		sdcontroller_.setPeriod(6); //period6

		//sdcontroller_.lr_cli=0.05;	
		//sdcontroller_.setPeriod_cli(4); //PER4 cli
		
		sdcontroller_.lr_cli1=0.05;
		sdcontroller_.setPeriod_cli1(6); //PER4 cli1

	}

//Syn
	if(counter_Switching == 15000)
	{

	//	sdcontroller_.memory = 100000;


		sdcontroller_.lr_=0.05;
		sdcontroller_.setPeriod(4); //period4

		//sdcontroller_.lr_cli=0.05;
		//sdcontroller_.setPeriod_cli(4); //PER4 cli
		
		sdcontroller_.lr_cli1=0.05;
		sdcontroller_.setPeriod_cli1(4); //PER4 cli1
	}

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
	sdcontroller_.stepcli();
	sdcontroller_.stepcli1();

	//Master
	outputChaosCPG1 = sdcontroller_.o1_;//Chaos control Output
	outputChaosCPG2 = sdcontroller_.o2_;//Chaos control Output

	outputChaosCPG1_Post = sdcontroller_.getOutput2(); //Post processing Output
	outputChaosCPG2_Post = sdcontroller_.getOutput1(); //Post processing Output


	
	//Client
	outputChaosCPG1cli = sdcontroller_.o1_cli;//Chaos control Output
	outputChaosCPG2cli = sdcontroller_.o2_cli;//Chaos control Output

	outputChaosCPG1cli_Post = sdcontroller_.getOutput2cli(); //Post processing Output
	outputChaosCPG2cli_Post = sdcontroller_.getOutput1cli(); //Post processing Output
	
	
	//Client1
	outputChaosCPG1cli1 = sdcontroller_.o1_cli1;//Chaos control Output
	outputChaosCPG2cli1 = sdcontroller_.o2_cli1;//Chaos control Output

	outputChaosCPG1cli1_Post = sdcontroller_.getOutput2cli1(); //Post processing Output
	outputChaosCPG2cli1_Post = sdcontroller_.getOutput1cli1(); //Post processing Output


	//Original controlled signals = "sdcontroller_.o1_ , sdcontroller_.o2_"
	//Converted signals with hysteresis and slope = "sdcontroller_.getOutput1() , sdcontroller_.getOutput2()"


	AverageChaosCPG = (outputChaosCPG1+outputChaosCPG2)/2;
    printf("CPG %d\n",counter_Switching);


	if(counter_Switching == 30000)
    exit(1);


saveFile1 <<outputChaosCPG1<<"  "<<outputChaosCPG2<<" "<<outputChaosCPG1_Post<<" "<<outputChaosCPG2_Post<<" "<<outputChaosCPG1cli<<" "<<outputChaosCPG2cli<<" "<<sdcontroller_.memory<< "   \n" << flush; //SAVE DATA 
	

}

