/**
 * @author Sakya & Poramate 02.12.2013
 */

#ifndef __TestESN_H__
#define __TestESN_H__

// #################### definitions #####################

#include <math.h>
#include <string.h>
#include <stdio.h>



///////// Save text////////////////////////
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
///////////////////////////////////////////


//Set parameters (for Students)
const int num_input_ESN = 1;
const int num_output_ESN = 1;
const int num_hidden_ESN = 100;

const int learning_mode = 1;
//set learning_mode = 1 for RLS (learning rate needs to be large, e.g., 0.99)
//set learning mode =2  for LMS (learning rate needs to be small, e.g., 0.01)

const double learning_rate_ESN = 0.99;
const double leak = 0.10;
const double input_sparsity = 70;

const int testing_start = 1500; /*training after, e.g., 1500 steps*/
//Total number of Data is 2000 data, ESN will be trained until 1499 steps and then it will be tested/, weights are fixed!/


class TestESN
{
  public:

    TestESN();
    ~TestESN();


    // --- Save text------------//
    ofstream saveFile1;
    //-------------------------//
    double RecurrentNetwork(double i0, double d);
    double target_ESN;
    double input_ESN;
    double output_ESN;
    bool learn;
    double error;


};

#endif
