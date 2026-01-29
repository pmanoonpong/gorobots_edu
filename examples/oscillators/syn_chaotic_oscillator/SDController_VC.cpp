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

//two neuron connection weights
	w11_ = -22.;
	w21_ = -6.6 ;
	w12_ = 5.9;
  
//two neuron bias terms
	theta1_ = -3.4;
	theta2_ = 3.8;

//learning rate
 	lr_ = 0.05; //0.05;//0.08;

//initial period
	setPeriod(1);

//control inputs
	input1_ = input2_ = 0;

//control factor
	cL_ =0;
  update_ = 2;
 

	/**************************** motor control **************************************/
	lin_[0].setInverse(+1.);
	lin_[1].setInverse(-1.);
	thetahys_[0] = -0.5;
	thetahys_[1] = -0.6;
}


SDController::~SDController()
{
	delete[] data_;
	delete[] DIRS_;
}


/** @brief sigmoid
  *
  * @todo: sigmoidal output
  */
double SDController::sigmoid_(double x)
{
	return 1./(1.+exp(-x));
}

/** @brief oscistep
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




/** @brief step
  *
  * @todo: call oscistep / targetinput and sets the steer
  */
void SDController::step()
{

//Main controller //////////////////////////////////////////////////////////////////////////////////////
//if counter equals goal period: reset counters, calculate diff-values, set targetinput for control

	if(n_== period_)
	{
		////////////CAL1///////////////////////////////////////////
		n_= 0;
		diff_n1_ = data_[period_].o1-data_[0].o1; //Output neuron1
		diff_n2_ = data_[period_].o2-data_[0].o2; //Output neuron2
		diffi_ = diff_n1_*diff_n1_+diff_n2_*diff_n2_;

		
    	sFile << diff_n1_<< ' ' << diff_n2_<< ' ' << data_[period_].o1<< ' ' << data_[0].o1 <<endl;

		targetinput();

	
		if(learning_)
				cL_+= diffi_*lr_/(period_);
		else{
				cL_-=0.2*cL_;
				printf("aha");
		}
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


	if(++percount%(normalize_)==0){
		lin_[0].step( FAC*(data_[n_].o1+thetahys_[0]) );
		lin_[1].step( FAC*(data_[n_].o2+thetahys_[1]) );
	}
	else
	{
		lin_[0].step( 0 );
		lin_[1].step( 0 );
	}

//	sFile << o1_ << ' ' << o2_ << ' ' << learning_<< ' ' << period_ <<endl;
// 	std::cerr << normalize_<< ' ' << percount <<std::endl;
}

void SDController::reset()
{
//	std::cerr << period_ << std::endl;
	percount = 0;

	cL_ =0;
  n_=0;
  data_[0].o1 = o1_ = 0;
  data_[0].o2 = o2_ = 0;
	normalize_ =  update_ * period_ +1;

	lin_[0].setSlope(2./normalize_, 2./((1-period_)*normalize_));
	lin_[1].setSlope(2./normalize_, 2./((1-period_)*normalize_));
	
  learning_ = true;

  printf("reset \n");
  
  // std::cerr << "normalize:  "<< normalize_<< "update:  "<< update_<< std::endl;
}


void SDController::setPeriod(int period)
{
	period_ = period;
	reset();
}

