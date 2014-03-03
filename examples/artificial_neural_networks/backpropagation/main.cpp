/*****************************************************************************
 *  Copyright (C) 2012 by Timo Nachstedt                                     *
 *                                                                           *
 *  This program is free software: you can redistribute it and/or modify     *
 *  it under the terms of the GNU General Public License as published by     *
 *  the Free Software Foundation, either version 3 of the License, or        *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                           *
 ****************************************************************************/

#include <utils/ann-framework/ann.h>
#include <utils/ann-framework/backpropagation.h>
#include <utils/ann-framework/neuron.h>
#include <iostream>
#include <map>

/**
 * "Batch mode learning"
 * Our feed-forward neural network for testing
 *
 *  Neurons 0,1,5 are input neurons
 *  Neurons 2,3 are hidden neurons
 *  Neuron 4 is the output neuron
 *
 *  Neuron 5 will be used as bias neuron with fixed input.
 *
 */
class TestANN : public ANN
{
  public:
    TestANN();
};

TestANN::TestANN() {

  //set transfer function
  //setDefaultTransferFunction(ANN::identityFunction());
  //setDefaultTransferFunction(ANN::tanhFunction());
  setDefaultTransferFunction(ANN::logisticFunction());
  //setDefaultTransferFunction(ANN::thresholdFunction());

  setNeuronNumber(5); // total number of neurons

// Set transfer function for individual neuron
//  n(0)->setTransferFunction(ANN::logisticFunction()); // input neuron
//  n(1)->setTransferFunction(ANN::identityFunction()); // input neuron
//  n(2)->setTransferFunction(ANN::logisticFunction()); // hidden neuron
//  n(3)->setTransferFunction(ANN::tanhFunction()); // hidden neuron
//  n(4)->setTransferFunction(ANN::logisticFunction()); // output neuron

  w(2,0, 0.1); // synapse from neuron 0 to 2 with weight 0.1
  w(2,1, 0.8);

  w(3,0,-1.0);
  w(3,1, 1.0);

  w(4,2, 0.1);
  w(4,3,-0.7);
}

int main(int argc, char **argv) {
  TestANN ann;

  // create a topolocial sorting of our network. This is required for the
  // backpropagation algorithm as "Full Batch mode"
  ann.updateTopologicalSort();

  // training data
  const double data[4][3] = {{0, 0, 0},
                             {0, 1, 1},
                             {1, 0, 1},
                             {1, 1, 0}}; // {input 1, input 2, output}

  // create backpropagation object
  Backpropagation trainer;
  trainer.setNeuralNetwork(&ann);
  trainer.defineInputNeuron(0, ann.getNeuron(0));
  trainer.defineInputNeuron(1, ann.getNeuron(1));
  trainer.defineOutputNeuron(0, ann.getNeuron(4));
  trainer.includeAllSynapses();
  trainer.includeAllNeuronBiases();
  trainer.setLearningRate(0.1);
  for (int i=0; i<4; i++)
  {
    TrainingPattern* p = new TrainingPattern;
    p->inputs[0]  = data[i][0]; // text file here from your own created inputs
    p->inputs[1]  = data[i][1];
    p->outputs[0] = data[i][2];
    trainer.addTrainingPattern(p);
  }

  trainer.learn(50000);

  ann.setInput(0,0);
  ann.setInput(1,0);
  ann.feedForwardStep();
  std::cout << "0 0 => " << ann.getOutput(4) << std::endl;

  ann.setInput(0,1);
  ann.setInput(1,0);
  ann.feedForwardStep();
  std::cout << "1 0 => " << ann.getOutput(4) << std::endl;

  ann.setInput(0,0);
  ann.setInput(1,1);
  ann.feedForwardStep();
  std::cout << "0 1 => " << ann.getOutput(4) << std::endl;

  ann.setInput(0,1);
  ann.setInput(1,1);
  ann.feedForwardStep();
  std::cout << "1 1 => " << ann.getOutput(4) << std::endl;

  std::cout << ann.dumpWeights();
}

