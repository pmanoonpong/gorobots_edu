#include "SDController.h"
#include <iostream>
#include <math.h>


SDController::SDController(): PERIODMAX(10), FAC(8)
{
sFile.open("neuron.txt",ios::out);
/**************************** two neuron network setup **************************************/
//outputs
  o1_=0;
  o2_=0;

//counter
	n_= 0;

//difference values
	diff_n1_=0;
	diff_n2_=0;
	diffi_ = 0;

//maximum period
	percount = 0;

//arrays for two neuron network output
  data_ = new data_t[PERIODMAX+1]; //(data_t*) calloc (periodmax_+1,sizeof(data_t));
  data_[0].o1 = 0;
  data_[0].o2 = 0;

//control matrix (unity)
	NDIRS_ = 4;
	DIRS_ = new int[NDIRS_]; //(int*) calloc (4,sizeof(int));
	DIRS_[0]=1;
	DIRS_[1]=0;

	DIRS_[2]=0;
	DIRS_[3]=1;

//learning rate
 	lr_ = 0.05; //0.05;//0.08;

//control factor
	cL_ =0;
    update_ = 2;


//control inputs
	input1_ = input2_ = 0;

//hyster inputs
	input_hys_0 = 0.0;
	input_hys_1 = 0.0;

	
//initial period
	setPeriod(1);

//************Cli0*******************************/	
//outputs
	o1_cli=0; 
    o2_cli=0; 
//counter
	n_cli = 0; 
  
//difference values
	diff_n1_cli=0;
	diff_n2_cli=0;
	diffi_cli = 0;

//maximum period
	percount_cli = 0;

//arrays for two neuron network output
  data_cli = new data_tcli[PERIODMAX+1]; //(data_t*) calloc (periodmax_+1,sizeof(data_t));
  data_cli[0].o1cli = 0;
  data_cli[0].o2cli = 0;

//control matrix (unity)
	NDIRS_cli = 4;
	DIRS_cli = new int[NDIRS_cli]; //(int*) calloc (4,sizeof(int));
	DIRS_cli[0]=1;
	DIRS_cli[1]=0;
	DIRS_cli[2]=0;
	DIRS_cli[3]=1;

//learning rate
	lr_cli = 0.05;

//control factor
	cL_cli =0;		 
    update_cli = 2;  
 
//control inputs

	input1_cli = input2_cli = 0; 

//hyster inputs
	input_hys_0_cli = 0.0;
	input_hys_1_cli = 0.0;

//initial period
	setPeriod_cli(1);

	
//************Cli1*******************************/	
//outputs
	o1_cli1=0; 
    o2_cli1=0; 
//counter
	n_cli1 = 0; 
  
//difference values
	diff_n1_cli1=0;
	diff_n2_cli1=0;
	diffi_cli1 = 0;

//maximum period
	percount_cli1 = 0;

//arrays for two neuron network output
  data_cli1 = new data_tcli1[PERIODMAX+1]; //(data_t*) calloc (periodmax_+1,sizeof(data_t));
  data_cli1[0].o1cli1 = 0;
  data_cli1[0].o2cli1 = 0;

//control matrix (unity)
	NDIRS_cli1 = 4;
	DIRS_cli1 = new int[NDIRS_cli1]; //(int*) calloc (4,sizeof(int));
	DIRS_cli1[0]=1;
	DIRS_cli1[1]=0;
	DIRS_cli1[2]=0;
	DIRS_cli1[3]=1;

//learning rate
	lr_cli1 = 0.05;

//control factor
	cL_cli1 =0;		 
    update_cli1 = 2;  
 
//control inputs

	input1_cli1 = input2_cli1 = 0; 

//hyster inputs
	input_hys_0_cli1 = 0.0;
	input_hys_1_cli1 = 0.0;

//initial period
	setPeriod_cli1(1);



	//************Motor control*******************************/	
	lin_[0].setInverse(+1.);
	lin_[1].setInverse(-1.);
	thetahys_[0] = -0.5;
	thetahys_[1] = -0.6;


	lin_cli[0].setInverse(+1.);
	lin_cli[1].setInverse(-1.);
	thetahys_cli[0] = -0.5;
	thetahys_cli[1] = -0.6;

	lin_cli1[0].setInverse(+1.);
	lin_cli1[1].setInverse(-1.);
	thetahys_cli1[0] = -0.5;
	thetahys_cli1[1] = -0.6;


	//************Global parameters***************************/
	//two neuron connection weights*****
		w11_ = -22.;
		w21_ = -6.6 ;
		w12_ = 5.9;
  
	//two neuron bias terms*****
		theta1_ = -3.4;
		theta2_ = 3.8;


	//************Syn_Memory parameters************************/
	//memory
	beta_steep = 1000;
	theta_steep = 10000; ////20000 time steps//////// Long or short delay

	//Memory
	memory = 100000; // Initial with Syn
	
}


SDController::~SDController()
{
	delete[] data_;
	delete[] DIRS_;

	delete[] data_cli;
	delete[] DIRS_cli;

	delete[] data_cli1;
	delete[] DIRS_cli1;
}


/** @brief sigmoid
  *
  * @todo: sigmoidal output
  */
double SDController::sigmoid_(double x)
{
	return 1./(1.+exp(-x));
}


/** @brief sigmoid
  *
  * @todo: sigmoidal output Steep
  */
double SDController::sigmoid_steep(double x)
{

	return 1./(1.+exp(-(beta_steep)*(theta_steep-x)));
}



/** @brief sigmoid
  *
  * @todo: sigmoidal output derivative steep
  */
double SDController::deri_sigmoid_steep(double x)
{
	double  deribeta_steep = 10;//1000;
	double  deritheta_steep = theta_steep; ////20000 time steps//////// Long or short delay

	//deri sig = f(x)(1-f(x))

	return (1./(1.+exp(-(deribeta_steep)*(deritheta_steep-x))))*(1-(1./(1.+exp(-(deribeta_steep)*(deritheta_steep-x)))));
}




//**************************************************************************************************/////
///------------------Master Oscillator Start --------------------------------////
/** @brief MASTER step
  *
  * @todo: call oscistep / targetinput and sets the steer
  */
//**************************************************************************************************/////


void SDController::step()
{

//Main controller ///////
	
	//if counter equals goal period: reset counters, calculate diff-values, set targetinput for control
	if(n_== period_)
	{
		////////////CAL1///////////////////////////////////////////
		n_= 0;
		diff_n1_ = data_[period_].o1-data_[0].o1; //Output neuron1
		diff_n2_ = data_[period_].o2-data_[0].o2; //Output neuron2
		diffi_ = diff_n1_*diff_n1_+diff_n2_*diff_n2_;

    	//sFile << diff_n1_<< ' ' << diff_n2_<< ' ' << data_[period_].o1<< ' ' << data_[0].o1 <<endl;


		targetinput();

	    // cL_= adapt
		/*	cL_= 0.044495; // p = 4
			cL_= 0.056449; // p = 5
			cL_= 0.012558; // p = 6 
			cL_= 0.017359; // p = 8
		*/
		if(learning_)
				cL_+= diffi_*lr_/(period_);
		else
				cL_-=0.2*cL_;
		if(!learning_){
			if(cL_<1e-6)
				learning_ = true;
		}

		/////////////////////////////////////////////////////////
	}
	//if counter does not equal goal period: usual oscistep is performed

	else{
		n_++;
	////////////CAL2///////////////////////////////////////////
		
		oscistep();

    data_[n_].o1 = o1_;
    data_[n_].o2 = o2_;

	}

///////////////////////////////////////////////////////////////////////////////////////////////////
///------------------Post processing (After chaos control) --------------------------------////
///------------------1) Shift signal -> data_[n_].o1 = output neuron with bias (theta1,2 = -0.5, -0.6)
///------------------2) Time window takes signals every -> normalize_ = 2*period + 1 interation, else = 0
///------------------3) Send signals to hysteresis elements SDHys::step() ;

//----------Master-Time Window Function---//

	if(++percount%(normalize_)==0){
		
		input_hys_0 = FAC*(data_[n_].o1+thetahys_[0]);
		input_hys_1 = FAC*(data_[n_].o2+thetahys_[1]);

		lin_[0].step(input_hys_0); //Output0 after the conversion of a time window function 
		//lin_[0] = 8*(o1_-0.5); // where 8 = connection weight to the hysteresis element, -0.5 = bias term
        // Then go to step() to calculate hysteresis output and the return here!!!!

		lin_[1].step(input_hys_1); //Output1 after the conversion of a time window function
		//lin_[1] = 8*(o2_-0.6); // where 8 = connection weight to the hysteresis element, -0.6 = bias term
        // Then go to step() to calculate hysteresis output and the return here!!!!
	}
	else
	{
		input_hys_0 = 0.0;
		input_hys_1 = 0.0;

		lin_[0].step(input_hys_0); //Output0 after the conversion of a time window function 
		lin_[1].step(input_hys_1); //Output1 after the conversion of a time window function
		
	}



//	sFile << o1_ << ' ' << o2_ << ' ' << learning_<< ' ' << period_ <<endl;
// 	std::cerr << normalize_<< ' ' << percount <<std::endl;
}


/** @brief oscistep 2 neurons system 
  *
  * @todo: calculate 2nn output
  */
void SDController::oscistep()
{

	// calculate output values
	double activityO1=w11_*o1_+w12_*o2_+theta1_+input1_;
	double activityO2=w21_*o1_+theta2_+input2_;

  
    o1_= sigmoid_(activityO1);
	o2_= sigmoid_(activityO2);

 // if targetinput is called, inputs must be reseted to zero
  input1_ = input2_ = 0;

}


/** @brief targetinput
  *
  * @todo: calculate the target input
  */
void SDController::targetinput()
{
  double d1 = 0, d2 = 0;

  // if learning is switched on: set control input
	if(learning_) {
		if(lr_){
			d1 = cL_*(DIRS_[0]*diff_n1_+DIRS_[1]*diff_n2_)-diff_n1_;
			d2 = cL_*(DIRS_[2]*diff_n1_+DIRS_[3]*diff_n2_)-diff_n2_;
			input1_ = w11_*d1 + w12_*d2;
			input2_ = w21_*d1;
		}
	}

	// do oscistep()
  oscistep();
  data_[0].o1 = o1_;
  data_[0].o2 = o2_;

}




//**************************************************************************************************/////
///------------------Client Oscillator Start --------------------------------////
/** @brief CLIENT step
  *
  * @todo: call oscistep / targetinput and sets the steer
  */
//**************************************************************************************************/////

void SDController::stepcli() //  Chapters 4 and 7
{


//**1************Reset to Period 1 when memory reach threshold!!!!

  	if(deri_sigmoid_steep(memory)>0)
	{
	  setPeriod_cli(1);
	}
//*********************





		//if counter equals goal period: reset counters, calculate diff-values, set targetinput for control
//	std::cerr << "period_cli:  "<<deri_sigmoid_steep(memory)<< std::endl;
//    sFile<<period_cli<<"  "<<deri_sigmoid_steep(memory)<<"   \n" << flush; //SAVE DATA 
	
	if(n_cli== period_cli) // period_ is set by sensor input conditions in SDcontroller()
	{
		n_cli= 0;// reset counter
		diff_n1_cli = data_cli[period_cli].o1cli-data_cli[0].o1cli; //Output neuron1 Last value of O1-O1 at time 0
		diff_n2_cli = data_cli[period_cli].o2cli-data_cli[0].o2cli; //Output neuron2 Last value of O2-O2 at time 0
		diffi_cli = diff_n1_cli*diff_n1_cli+diff_n2_cli*diff_n2_cli;

		

		////Period detection mechanism//////////
		
	    //Still not working
		
			if (rand()/((double)RAND_MAX) < diffi_cli) 
				period_cli += (int)((rand()%3)-1)*(1-sigmoid_steep(memory));

			if(period_cli < 1)
				period_cli = 1;
		
		//sFile<<period_cli<<"  "<<(int)((rand()%3)-1)  <<"   \n" << flush; //SAVE DATA 
		
		
		
		///////////////////////////////////////

		targetinput_cli();
	
		//cL_cli = fixed value
		//Take cL_ and period of Master to set the client!!!!
			/*	cL_= 0.044495; // p = 4
				cL_= 0.056449; // p = 5
				cL_= 0.012558; // p = 6 
				cL_= 0.017359; // p = 8
			*/

		if(learning_cli)
				cL_cli+= diffi_cli*lr_cli/(period_cli); // eq. 4.28
		else
				cL_cli-=0.2*cL_cli;

		
		/////////Adaptation//////////////////////
		
		
		///////////////Memory///////////
		cL_cli = cL_cli*sigmoid_steep(memory);

		//std::cerr << "alpha_syn:  "<< cL_cli<< std::endl;
		///////////////Memory///////////

	
		if(!learning_){
			if(cL_cli<1e-6)
				learning_cli = true;
		}
		///////////////////////

	}
	//if counter does not equal goal period: usual oscistep is performed

	else{
		n_cli++;

	oscistep_cli();

    data_cli[n_cli].o1cli = o1_cli;
    data_cli[n_cli].o2cli = o2_cli;

	}


///////////////////////////////////////////////////////////////////////////////////////////////////
///------------------Post processing (After chaos control) --------------------------------////
///------------------1) Shift signal -> data_[n_].o1 = output neuron with bias (theta1,2 = -0.5, -0.6)
///------------------2) Time window takes signals every -> normalize_ = 2*period + 1 interation, else = 0
///------------------3) Send signals to hysteresis elements SDHys::step() ;



	//----------Cli0-Time Window Function---//
	if(++percount_cli%(normalize_cli)==0){
 
		lin_cli[0].step(FAC*(data_cli[n_cli].o1cli+thetahys_cli[0])); //Output0 after the conversion of a time window function 
		lin_cli[1].step(FAC*(data_cli[n_cli].o2cli+thetahys_cli[1])); //Output1 after the conversion of a time window function
		}
	else
	{
		lin_cli[0].step(0); //else   give 0 as input to hysteresis elemnet where output hysteresis = -1
		lin_cli[1].step(0); //else   give 0 as input to hysteresis elemnet where output hysteresis = -1
	}

    //----------End Time Window Function ::: Got Output O_hys,1,2 with time window and hysteresis effects = lin_[0], lin_[1] = Figure 9.12 Output O_hys,2*********//

}



/** @brief oscistep 2 neurons system 
  *
  * @todo: calculate 2nn output
  */
void SDController::oscistep_cli()
{

//	memory = 0; // alpha_syn = 0 = NOT SYN
//	memory = 10000;   // alpha_syn = 0.5
//	memory > 10000;   // alpha_syn = 1 = SYN

	alpha_syn = 1*(1-sigmoid_steep(memory));


//Only Syn not CL control    
	double activityO1_cli=w11_*o1_cli+w12_*o2_cli+theta1_+input1_cli;
    double activityO2_cli=w21_*o1_cli+theta2_+input2_cli; 
    
	    
	//double activityO1_cli=w11_*o1_cli+w12_*o2_cli+theta1_+input1_cli;
    //double activityO2_cli=w21_*o1_cli+theta2_+input2_cli; 
    
	
//	o1_cli= sigmoid_(activityO1_cli);
//	o2_cli= sigmoid_(activityO2_cli)+alpha_syn*(o2_- sigmoid_(activityO2_cli)); //-------SYN NOT WORK for all


	o1_cli= sigmoid_(activityO1_cli)+alpha_syn*(o1_- sigmoid_(activityO1_cli)); //-------SYN WORK for all
	o2_cli= sigmoid_(activityO2_cli); 


    //o2_cli= alpha_syn*o2_ + (1-alpha_syn)*sigmoid_(activityO2_cli);
	//o2_cli= sigmoid_(activityO2_cli);

	// if targetinput is called, inputs must be reseted to zero
    input1_cli = input2_cli = 0;

	
	sFile<<alpha_syn<<"  "<<memory<<" "<<deri_sigmoid_steep(memory)<<" "<<period_cli<<" "<<period_<<" "<<cL_cli<<"   \n" << flush; //SAVE DATA 

	  

   // std::cerr << "alpha_syn:  "<< alpha_syn<< std::endl;
}

/** @brief targetinput
  *
  * @todo: calculate the target input
  */
void SDController::targetinput_cli()
{
   double d1_cli = 0, d2_cli = 0;

  // if learning is switched on: set control input
	if(learning_) {
		if(lr_cli){

			d1_cli = cL_cli*diff_n1_cli-diff_n1_cli;  //----(3)
			d2_cli = cL_cli*diff_n2_cli-diff_n2_cli;  //----(3)

			input1_cli = w11_*d1_cli + w12_*d2_cli;
			input2_cli = w21_*d1_cli;
		}
	}

  oscistep_cli();
  data_cli[0].o1cli = o1_cli;
  data_cli[0].o2cli = o2_cli;

}



//**************************************************************************************************/////
///------------------Client1 Oscillator Start --------------------------------////
/** @brief CLIENT1 step
  *
  * @todo: call oscistep / targetinput and sets the steer
  */
//**************************************************************************************************/////

void SDController::stepcli1() //  Chapters 4 and 7
{


//**************CLIENT_1 cL_cli will be fixed!!!!!

//if counter equals goal period: reset counters, calculate diff-values, set targetinput for control

	if(n_cli1== period_cli1) // period_ is set by sensor input conditions in SDcontroller()
	{
		n_cli1= 0;// reset counter
		diff_n1_cli1 = data_cli1[period_cli1].o1cli1-data_cli1[0].o1cli1; //Output neuron1 Last value of O1-O1 at time 0
		diff_n2_cli1 = data_cli1[period_cli1].o2cli1-data_cli1[0].o2cli1; //Output neuron2 Last value of O2-O2 at time 0
		diffi_cli1 = diff_n1_cli1*diff_n1_cli1+diff_n2_cli1*diff_n2_cli1;

		targetinput_cli1();

	    //cL_cli1 = fixed value
		if(learning_cli1)
				cL_cli1+= diffi_cli1*lr_cli1/(period_cli1); // eq. 4.28
		else
				cL_cli1-=0.2*cL_cli1;
		if(!learning_cli1){
			if(cL_cli1<1e-6)
				learning_cli1 = true;
		}
		///////////////////////

	}
//if counter does not equal goal period: usual oscistep is performed

	else{
		n_cli1++;

		oscistep_cli1();
    data_cli1[n_cli1].o1cli1 = o1_cli1;
    data_cli1[n_cli1].o2cli1 = o2_cli1;

	}

///------------------Post processing (After chaos control) --------------------------------////
///------------------1) Shift signal -> data_[n_].o1 = output neuron with bias (theta1,2 = -0.5, -0.6)
///------------------2) Time window takes signals every -> normalize_ = 2*period + 1 interation, else = 0
///------------------3) Send signals to hysteresis elements SDHys::step() ;

	//----------Cli0-Time Window Function---//
	if(++percount_cli1%(normalize_cli1)==0){
 
		lin_cli1[0].step(FAC*(data_cli1[n_cli1].o1cli1+thetahys_cli1[0])); //Output0 after the conversion of a time window function 
		lin_cli1[1].step(FAC*(data_cli1[n_cli1].o2cli1+thetahys_cli1[1])); //Output1 after the conversion of a time window function
		}
	else
	{
		lin_cli1[0].step(0); //else   give 0 as input to hysteresis elemnet where output hysteresis = -1
		lin_cli1[1].step(0); //else   give 0 as input to hysteresis elemnet where output hysteresis = -1
	}

	//----------End Time Window Function ::: Got Output O_hys,1,2 with time window and hysteresis effects = lin_[0], lin_[1] = Figure 9.12 Output O_hys,2*********//

}

void SDController::oscistep_cli1()
{

	// calculate output values
	double activityO1_cli1=w11_*o1_cli1+w12_*o2_cli1+theta1_+input1_cli1;
	double activityO2_cli1=w21_*o1_cli1+theta2_+input2_cli1;

    o1_cli1= sigmoid_(activityO1_cli1);
	o2_cli1= sigmoid_(activityO2_cli1);

 // if targetinput is called, inputs must be reseted to zero
    input1_cli1 = input2_cli1 = 0;

//fprintf( stream8, "%f %f \n", o1_cli1, o2_cli1);

}

/** @brief targetinput
  *
  * @todo: calculate the target input
  */
void SDController::targetinput_cli1()
{
 
   double d1_cli1 = 0, d2_cli1 = 0;

  // if learning is switched on: set control input
	if(learning_) {
		if(lr_cli1){
		//	d1_cli1 = cL_cli1*(DIRS_cli1[0]*diff_n1_cli1+DIRS_cli1[1]*diff_n2_cli1)-diff_n1_cli1;
		//	d2_cli1 = cL_cli1*(DIRS_cli1[2]*diff_n1_cli1+DIRS_cli1[3]*diff_n2_cli1)-diff_n2_cli1;
		
			d1_cli1 = cL_cli1*diff_n1_cli1-diff_n1_cli1;  //----(3)
			d2_cli1 = cL_cli1*diff_n2_cli1-diff_n2_cli1;  //----(3)

			input1_cli1 = w11_*d1_cli1 + w12_*d2_cli1;
			input2_cli1 = w21_*d1_cli1;
		}
	}


  oscistep_cli1();

  data_cli1[0].o1cli1 = o1_cli1;
  data_cli1[0].o2cli1 = o2_cli1;

}



////////////Call only one time when change to new period//////////////////////

//***********MASTER

void SDController::setPeriod(int period)
{
	period_ = period;
	reset();
}

void SDController::reset()
{
//	std::cerr << period_ << std::endl;
	percount = 0;

	cL_ =0;
	n_=0;
	data_[0].o1 = o1_ = 0;
	data_[0].o2 = o2_ = 0;


//***********RESET**********************************************************************************/////
///------------------Post processing (After chaos control) --------------------------------////
///------------------1) Step up down slope period
///------------------2) Call SDHys::setSlope() ;
//**************************************************************************************************/////

	//******RESET*The up and down slopes are calculated **

	normalize_ =  update_ * period_ +1;

	lin_[0].setSlope(2./normalize_, 2./((1-period_)*normalize_));
	lin_[1].setSlope(2./normalize_, 2./((1-period_)*normalize_));
	
	learning_ = true;

	//printf("reset \n");
  
  // std::cerr << "normalize:  "<< normalize_<< "update:  "<< update_<< std::endl;
}


//***********Client0

void SDController::setPeriod_cli(int periodcli)
{
	//Cli0
	period_cli = periodcli;//period;
	reset_cli();
}

void SDController::reset_cli()
{
//	std::cerr << period_ << std::endl;
	
	//////CLI0
	percount_cli = 0;

	cL_cli =0;
	n_cli=0;
	data_cli[0].o1cli = o1_cli = 0;
	data_cli[0].o2cli = o2_cli = 0;

//************RESET*********************************************************************************/////
///------------------Post processing (After chaos control) --------------------------------////
///------------------1) Step up down slope period
///------------------2) Call SDHys::setSlope() ;
//**************************************************************************************************/////

	//********The up and down slopes are calculated **

	normalize_cli =  update_cli * period_cli +1; //(2p+1)
    //Cli
	lin_cli[0].setSlope(2./normalize_cli, 2./((1-period_cli)*normalize_cli));
	lin_cli[1].setSlope(2./normalize_cli, 2./((1-period_cli)*normalize_cli));

	learning_cli = true;
}


//***********Client1

void SDController::setPeriod_cli1(int periodcli1)
{
	//Cli1
	period_cli1 = periodcli1;//period;
	reset_cli1();
}

void SDController::reset_cli1()
{
//	std::cerr << period_ << std::endl;
	
	//////CLI0
	percount_cli1 = 0;

	cL_cli1 =0;
	n_cli1=0;
	data_cli1[0].o1cli1 = o1_cli1 = 0;
	data_cli1[0].o2cli1 = o2_cli1 = 0;

//************RESET*********************************************************************************/////
///------------------Post processing (After chaos control) --------------------------------////
///------------------1) Step up down slope period
///------------------2) Call SDHys::setSlope() ;
//**************************************************************************************************/////

	//********The up and down slopes are calculated **

	normalize_cli1 =  update_cli1 * period_cli1 +1; //(2p+1)
    //Cli
	lin_cli1[0].setSlope(2./normalize_cli1, 2./((1-period_cli1)*normalize_cli1));
	lin_cli1[1].setSlope(2./normalize_cli1, 2./((1-period_cli1)*normalize_cli1));

	//*******End The up and down slopes are calculated. Return m_1,_2(t) = lin_[0] and lin_[1] **************////
	learning_cli1 = true;
}


////////////Call only one time when change to new period//////////////////////