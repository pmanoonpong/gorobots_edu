/*
 * NeuralLearningAndMemoryYourExtension.h
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 */

#ifndef NEURALLEARNINGANDMEMORYYOUREXTENSION_H_
#define NEURALLEARNINGANDMEMORYYOUREXTENSION_H_


#include <vector>
#include <cmath>
#include <ode_robots/amosiisensormotordefinition.h>
//#include <selforg/amosiisensormotordefinition.h>
//#include "sensor_motor_definition.h"


//Save files
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
//Save files

  //2) Class for Neural learning and memory-----


  class NeuralLearningAndMemoryYourExtension{
  public:

	 //---Start Define functions---//

	NeuralLearningAndMemoryYourExtension();
	~NeuralLearningAndMemoryYourExtension();
	std::vector<double> step_nlm(const std::vector<double> in0);

	 //---End Define functions---//

	// add public available variables

	//Save files
	ofstream outFilenlm1;
	//Save files

  private:
  	// add private  variables
 };



#endif /* NEURALLEARNINGANDMEMORYYOUREXTENSION_H_ */
