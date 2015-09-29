#include "ico.h"

#include <utils/ann-framework/ann.h>
#include <utils/ann-framework/neuron.h>
#include <vector>
#include <iterator>
#include <iostream>

// default constructor
ICO::ICO(){
  // change default transfer function to linear
  ANN::setDefaultTransferFunction(ANN::identityFunction());

  rate_ico=0.1;
  oldReflexiveInput=0;

  // create two input neurons, reflexive and predictive
  for(int i=0; i<2; i++){
    inputNeurons.push_back(addNeuron());
  }

  // create output neuron
  outputNeuron=addNeuron();

  // create synapses between the neurons
  w(outputNeuron, inputNeurons[0], 1.0);
  w(outputNeuron, inputNeurons[1], 0.0);
}

ICO::ICO(const double& rate){
  // change default transfer function to linear
  ANN::setDefaultTransferFunction(ANN::identityFunction());

  rate_ico=rate;
  oldReflexiveInput=0;

  // create two input neurons, reflexive and predictive
  for(int i=0; i<2; i++){
    inputNeurons.push_back(addNeuron());
  }

  // create output neuron
  outputNeuron=addNeuron();

  // create synapses between the neurons
  w(outputNeuron, inputNeurons[0], 1.0);
  w(outputNeuron, inputNeurons[1], 0.0);
}

ICO::ICO(const int& neuronNumber, const double& rate){
  // change default transfer function to linear
  ANN::setDefaultTransferFunction(ANN::identityFunction());

  rate_ico=rate;
  oldReflexiveInput=0;

  // create two input neurons, reflexive and predictive
  for(int i=0; i<neuronNumber+1; i++){
    inputNeurons.push_back(addNeuron());
  }

  // create output neuron
  outputNeuron=addNeuron();

  // create synapses between the neurons
  w(outputNeuron, inputNeurons[0], 1.0);
  w(outputNeuron, inputNeurons[1], 0.0);
}

ICO::~ICO(){}

// step overwrite for ICO
void ICO::step(){
  updateActivities();
  updateOutputs();
  updateWeights();
}

void ICO::updateWeights(){
  NeuronList::iterator it=inputNeurons.begin();
  std::advance(it,1);
  for(; it!=inputNeurons.end(); it++){
    double weight;
    weight=rate_ico*(inputNeurons[0]->getInput()-oldReflexiveInput)*(*it)->getInput();
    setWeight(outputNeuron, (*it), weight+getWeight(outputNeuron, (*it)));
  }
}

void ICO::setInputNeuronInput(const int& index, const double& value){
  if(index==0){
    oldReflexiveInput=inputNeurons[0]->getInput();
  }

  setInput(inputNeurons[index], value);
}

const double& ICO::getOutputNeuronOutput(){
  return getOutput(outputNeuron);
}

void ICO::setReflexiveNeuronInput(const double& value){
  setInputNeuronInput(0, value);
}

void ICO::setPredictiveNeuronInput(const int& index, const double& value){
  setInputNeuronInput(index+1, value);
}

void ICO::setPredictiveNeuronInput(std::vector<double>& values){
  if(values.size()==getNeuronNumber()-2){
    for(int i=0; i<values.size(); i++){
      setInputNeuronInput(i+1, values[i]);
    }
    return ;
  }
  std::cerr << "Wrong size of neuron input vector." << std::endl;
}

