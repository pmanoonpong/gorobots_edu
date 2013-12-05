/**
 * @author Sakya & Poramate 02.12.2013
 */

#include "esntest.h"
#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


//Add ENS network--(1)
#include "utils/esn-framework/networkmatrix.h"
//-----ESN network-----//
ESNetwork * ESN;
float * ESinput;
float * ESTrainOutput;



// ----------------------------------------------------------------------
// ------------ Initial constructor -------------------------------------
// ----------------------------------------------------------------------

TestESN::TestESN()
{

  srand (time(NULL));

  //--------------------------Add ENS network--(2)-----------------------------------//

  ESN = new ESNetwork(num_input_ESN/*no. input*/,num_output_ESN /*no. output*/, num_hidden_ESN /*rc hidden neurons*/, false /*feedback*/, false /*feeding input to output*/, leak /*0.1 leak = 0.0-1.0*/, false /*false IP*/);
  ESN->outnonlinearity = 0; //0 = linear, 1 = sigmoid, 2  = tanh: transfer function of an output neuron
  ESN->nonlinearity = 2; //0 = linear, 1 = sigmoid, 2  = tanh: transfer function of all hidden neurons
  ESN->withRL = 2; //2 = stand ESN learning, 1 = RL with TD learning

  ESN->InputSparsity = input_sparsity; //if 0 = input connects to all hidden neurons, if 100 = input does not connect to hidden neurons
  ESN->autocorr = pow(10,4); //set as high as possible, default = 1
  ESN->InputWeightRange = 0.1; // scaling of input to hidden neurons, default 0.15 means [-0.15, +0.15]
  ESN->LearnMode = learning_mode; //RLS = 1 (learning rate needs to be large, 0.99). LMS =2 (learning rate needs to be very small, e.g., 0.01)
  ESN->Loadweight = false; // true = loading learned weights
  ESN->NoiseRange = 0.001; // amplitude of noise
  ESN->RCneuronNoise = false; // false = constant fixed bias, true = changing noise bias every time

  ESN->generate_random_weights(50 /*70  10% sparsity = 90% connectivity */, 0.95 /*1.2-1.5 = chaotic*/);


  //Create ESN input vector
  ESinput = new float[num_input_ESN];
  //Create ESN target output vector
  ESTrainOutput = new float[num_output_ESN];

  //Initial values of input and target output
  for(unsigned int i = 0; i < num_input_ESN; i++)
  {
    ESinput[i] = 0.0;
  }

  for(unsigned int i = 0; i< num_output_ESN; i++)
  {
    ESTrainOutput[i] = 0.0;
  }

  //--------------------------Add ENS network--(2)-----------------------------------//

  saveFile1.open("result.txt",ios::out);

}

// ----------------------------------------------------------------------
// ------------ destructor ----------------------------------------------
// ----------------------------------------------------------------------

TestESN::~TestESN()
{
  delete []ESN;
  delete []ESinput;
  delete []ESTrainOutput;

}

// ----------------------------------------------------------------------
// --- Recurrent Neural Networks  ---------------------------------------
// ----------------------------------------------------------------------
double TestESN::RecurrentNetwork (double i0, double d)
{
  learn = true;
  static int interation = 0;

  interation++;

  if (interation>testing_start)
  {
    learn = false;
    std::cout<<"**********testing mode***********"<< "\n";
  }
  else
  {
    std::cout<<"--training mode--"<< "\n";
  }


  ESTrainOutput[0]= d;//target_ESN; //Set Target output to ESN

  ESinput[0] = i0;//input_ESN;// Set Input to ESN

  ESN->setInput(ESinput, num_input_ESN/* no. input*/); // Call ESN

  //ESN Learning function
  ESN->takeStep(ESTrainOutput, learning_rate_ESN /*0.9 RLS*/, 1 /*no td = 1 else td_error*/, learn/* true= learn, false = not learning learn_critic*/, 0);

  output_ESN = ESN->outputs->val(0, 0);//Read out the output of ESN

  ESN->printMatrix(ESN->endweights); //print weight matrix on screen



  saveFile1 <<ESinput[0]<<"  "<<ESTrainOutput[0]<<"  "<<output_ESN<<"   \n" << flush; //SAVE DATA

}


