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
  setNeuronNumber(6); // total number of neurons

  w(2,0, 0.1); // synapse from neuron 0 to 2 with weight 0.1
  w(2,1, 0.8);
  w(2,5,-0.5);

  w(3,0,-1.0);
  w(3,1, 1.0);
  w(3,5, 0.5);

  w(4,2, 0.1);
  w(4,3,-0.7);
  w(4,5, 0.3);
}

int main(int argc, char **argv) {
  TestANN ann;

  // set bias neuron to 1
  ann.setInput(5,1);

  // create a topolocial sorting of our network. This is required for the
  // backpropagation algorithm
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
  trainer.setLearningRate(0.2);
  for (int i=0; i<4; i++)
  {
    TrainingPattern* p = new TrainingPattern;
    p->inputs[0]  = data[i][0]; // text file here from your own created inputs
    p->inputs[1]  = data[i][1];
    p->outputs[0] = data[i][2];
    trainer.addTrainingPattern(p);
  }

  trainer.learn(10000);

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
}

