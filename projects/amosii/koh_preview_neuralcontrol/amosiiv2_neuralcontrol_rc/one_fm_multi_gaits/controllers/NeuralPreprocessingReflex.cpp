/*
 * NeuralPreprocessingReflex.cpp
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 */

#include "NeuralPreprocessingReflex.h"


///-------------------------------------------------------------------------------------------------------------------------

//1) Class for Neural preprocessing------------


NeuralPreprocessingReflex::NeuralPreprocessingReflex(){

		 //Save files
		 outFilenpp1.open("Neuralpreprocessing.dat");

		 //---Set vector size----//
		 mappingsensor.resize(AMOSII_SENSOR_MAX);
		 sensor_activity.resize(AMOSII_SENSOR_MAX);
		 sensor_output.resize(AMOSII_SENSOR_MAX);
		 preprosensor.resize(AMOSII_SENSOR_MAX);

  		 //---Initialize your values

//		 sensor_w_pfs_rfs = 1.0;//5.0;
//		 sensor_w_pfs_pfs = 1.2;//2.0;

		   sensor_w_pfs_rfs = 5.0;
	     sensor_w_pfs_pfs = 2.0;

  	};

NeuralPreprocessingReflex::~NeuralPreprocessingReflex(){

		  //Save files
		  outFilenpp1.close();

	 };

//1)  Step function of Neural preprocessing------------
std::vector<double> NeuralPreprocessingReflex::step_npp(const std::vector<double> in0){

	//----------KOH--------//

	//Define local parameters

	//1)****Prepro Foot sensors*********//
	double Amplifiy_factor_Foot = 2.5;
	int signal_inversion = -1;
	double average_current_sensor;


	//1)****Prepro Foot sensors for searching reflexes********

	for(unsigned int i=R0_fs; i<(L2_fs+1);i++)
	{
		//Mapping foot sensor to +-1
		mappingsensor.at(i)=(((in0.at(i)-1)/(0-1))*2.0-1.0);

		bool step;

		step = false;
		if(step)
		{
		  std::cout<<"Step Foot sensor signals"<<std::endl;

			if (mappingsensor.at(i)>0.9)
			{
				mappingsensor.at(i)=1;
			}
			if (mappingsensor.at(i)<=0.9)
			{
				mappingsensor.at(i)=-1;
			}

			//Preprocessing
			sensor_activity.at(i) = mappingsensor.at(i)*sensor_w_pfs_rfs+sensor_output.at(i)*sensor_w_pfs_pfs;//*presyFL3+biasFL3+ac_OutPostprocessFL3*recurrentFL3;
			sensor_output.at(i) = tanh(sensor_activity.at(i));
			preprosensor.at(i) = sensor_output.at(i);
		}

		//preprosensor.at(i) = mappingsensor.at(i);


		if(!step)
		{
		  double weight = 0.8;

		  std::cout<<"Lowpass Raw Foot sensor signals"<<std::endl;

		  sensor_activity.at(i) = mappingsensor.at(i)*(1-weight)+sensor_output.at(i)*weight;
		  sensor_output.at(i) = sensor_activity.at(i);
		  preprosensor.at(i) = sensor_output.at(i);
		}

	}

	average_current_sensor = in0.at(A_cs);

	  // >> i/o operations here <<

  // >> i/o operations here <<
  outFilenpp1<<in0.at(R0_fs)<<' '<<mappingsensor.at(R0_fs)<<' '<<sensor_output.at(R0_fs)<<' '<<preprosensor.at(R0_fs)<<' '
             <<in0.at(R1_fs)<<' '<<mappingsensor.at(R1_fs)<<' '<<sensor_output.at(R1_fs)<<' '<<preprosensor.at(R1_fs)<<' '
             <<in0.at(R2_fs)<<' '<<mappingsensor.at(R2_fs)<<' '<<sensor_output.at(R2_fs)<<' '<<preprosensor.at(R2_fs)<<' '
             <<in0.at(L0_fs)<<' '<<mappingsensor.at(L0_fs)<<' '<<sensor_output.at(L0_fs)<<' '<<preprosensor.at(L0_fs)<<' '
             <<in0.at(L1_fs)<<' '<<mappingsensor.at(L1_fs)<<' '<<sensor_output.at(L1_fs)<<' '<<preprosensor.at(L1_fs)<<' '
             <<in0.at(L2_fs)<<' '<<mappingsensor.at(L2_fs)<<' '<<sensor_output.at(L2_fs)<<' '<<preprosensor.at(L2_fs)<<' '
             <<endl;

	//
	//---------------------------//



	return preprosensor;

};

///-------------------------------------------------------------------------------------------------------------------------




