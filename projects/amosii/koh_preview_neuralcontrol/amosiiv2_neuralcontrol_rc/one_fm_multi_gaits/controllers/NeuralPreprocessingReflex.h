/*
 * NeuralPreprocessingReflex.h
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 */

#ifndef NEURALPREPROCESSINGREFLEX_H_
#define NEURALPREPROCESSINGREFLEX_H_

#include <vector>
#include <cmath>
#include <ode_robots/amosiisensormotordefinition.h>



//Save files
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
//Save files

  //1) Class for Neural preprocessing------------


  class NeuralPreprocessingReflex{

  public:

	 //---Start Define functions---//
	  NeuralPreprocessingReflex();
	  ~NeuralPreprocessingReflex();
  	 std::vector<double> step_npp(const std::vector<double> in0);

  	 //---End Define functions---//

  	 //---Start Define vector----//
  	 // add public available variables
  	 std::vector<double> mappingsensor;
  	 std::vector<double> sensor_activity;
  	 std::vector<double> sensor_output;
  	 std::vector<double> preprosensor;
  	 //---End Define vector----//


  	 //Save files
  	 ofstream outFilenpp1;
  	 //Save files


  private:
  	// add private  variables
	   double sensor_w_pfs_rfs;
	   double sensor_w_pfs_pfs;

 };



#endif /* NEURALPREPROCESSINGREFLEX_H_ */

