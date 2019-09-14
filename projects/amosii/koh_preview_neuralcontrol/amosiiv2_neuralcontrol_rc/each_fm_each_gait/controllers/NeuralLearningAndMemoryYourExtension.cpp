/*
 * NeuralLearningAndMemoryYourExtension.cpp
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 */

#include "NeuralLearningAndMemoryYourExtension.h"

//2) Step function of Neural learning and memory-----


NeuralLearningAndMemoryYourExtension::NeuralLearningAndMemoryYourExtension(){

	//Save files
	outFilenlm1.open("Neurallearning_memory.dat");

	//---Initialize your values

};

NeuralLearningAndMemoryYourExtension::~NeuralLearningAndMemoryYourExtension(){

	//Save files
	outFilenlm1.close();

};


std::vector<double> NeuralLearningAndMemoryYourExtension::step_nlm(const std::vector<double> in0){
	// just a silly example, do your neural learning stuff here and generate an output vector
	std::vector<double> output;
	double tmp;
	for (int i=0; i<(int)in0.size();i++){
		tmp+=in0.at(i);
	}
	for (int i=TR0_as; i<=TL2_as;i++){
		tmp+=in0.at(i);
	}

  // >> i/o operations here <<
  outFilenlm1<<tmp<<' '<<tmp<<endl;


	output.push_back(tmp);
	return output;
};
