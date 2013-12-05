/*
 * networkmatrix.h
 *
 *  Created on: Mar 22, 2012
 *  Last Modified on: Aug 13,2013
 *      Authors: Andrej fillipow and Sakyasingha Dasgupta
 *
 *      Please refer to Readme for basic details
 */

#ifndef NETWORKMATRIX_H_
#define NETWORKMATRIX_H_
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include "selforg/matrixutils.h"
#include "selforg/matrix.h"
#include <string>




class ESNetwork{
  public :
    /*
     * The following functions have to be called manually, and are necessary for functioning
     * */


    ESNetwork(int a, int b, int c, bool param1 = false, bool param2 = false, double param3 = 0.3, bool param4= false);

    /**
     * Generate the ESN network with all internal parameters defined
     * Takes the number of input, output and hidden neurons, whether inputs should feed into outputs, whether outputs
     * should feed back, and how quickly neurons should leak, in that order. Remember to initialize the weights!
     *
     **/


    ~ESNetwork();

    /**
     * Default destructor - handle or dangling objects
     **/

    void generate_random_weights(int sparsity = 90 /*95*/, float spectral_radius = 0.95 /*0.95*/);

    /**
     * Generating function for the input, reservoir and output weights. Sparsity governs percentage of matrix sparseness.
     * Inputweights drawn randomnly from the uniform distribution [-0.5,0.5). Innerweights drawn randomnly from the uniform
     * distribution [-1.1). Output weights are all initialised to 0. Innerweights are subsequently scaled to the specific
     * spectral_radius
     **/


    void generate_neighbour_weights(int sparsity = 90, float spectral_radius = 0.95, int nNextNeighbours = 15);

    /**
     * Same as above except with full random Neighboured weighting
     **/



    void setInput(float *Input, int size);

    /**
     * Accepts an array of floats, of length size, and sets the inputs to these given values. can use resetInput()
     * or scrambleInput() instead
     */


    void trainOutputs(float * Inputs, float * Outputs, int time, int discardedTimesteps = 30);

    /**
     * Offline training of the output weights. The net is trained over specific "time" steps, the first "discardedTimesteps" time steps are
     * not used to train. This serves as warm up for the ESN network.
     **/


    void trainOnlineRecursive(float * Outputs, float forgettingFactor, float td_error, double param = 0.0);

    /**
     *  Online training of the output weights using the Recursive least squares algorithm. For algorithm introduction
     *  check http://en.wikipedia.org/wiki/Recursive_least_squares_filter
     *
     **/

    void trainOnlineLMS(float * Outputs, float forgettingFactor, float td_error, double param = 0.0);

    /*
     * Least Means squared learning of Endweights
     */

    void trainBackpropagationDecorrelation(float * Outputs, float learningRate);
    /*
     * Backpropagation-Decorrelation learning function. This is intended for online learning purposes. Note that this is not fully functional yet.
     */

    void takeStep(float * Outputs, float learningRate, float td_error, bool learn = true, int timestep = 1, double param = 0.0);
    /*
     * Called at every discrete time step, calculating the inner neuron activations and outputs for the next discrete timestep.
     * If "learn_online" is true, the ESN learns the outputs specified in
     * "Outputs" via "trainOnlineRecursive()"
     * NOTE: the learningRate parameter acts as the forgetting factor for the trainOnlineRecursive() function.
     *
     */
    void takeStep()
      {
      float nothing[0];
      takeStep(nothing, 1, 0, false);
      }
    /*
     * The ESN takes a step without online learning, as above. this is called e.g. from "trainOutputs()"
     */

    /*
     * The following functions have to be called manually, and are optional
     * */


    float evaluatePerformance(int start,int end, float * desiredOutputs);
    /*
     * returns the mean square error between given Outputs and computed outputs over "time" discrete timesteps. Please refer to readme for further details.
     */

    void writeInneractivityToFile(int num);

    void writeNoiseToFile(int num);

    void writeInnerweightsToFile(int num);

    void writeStartweightsToFile(int num);

    void writeEndweightsToFile(int num);
    /*
     * Write the Endweights (RC to output neurons) values to a text file
     */

    void readInnerweightsFromFile(int num);

    void readStartweightsFromFile(int num);

    void readEndweightsFromFile(int num);

    void readNoiseFromFile(int num);

    void resetInput();
    /*
     * resets input to zero
     */
    void scrambleInput();
    /*
    * randomizes input
    */
    void printMatrix(matrix::Matrix *printedMatrix);
    /*
    * prints the passed matrix to the terminal
    */

    void printOutputToTerminal();
    /*
     * prints the outputs to the terminal
     */
    void printNeuronsToTerminal();
    /*
    * This function prints the activations of the inner neurons to the terminal
    */

    double uniform(double a, double b);
    /*
     *  generate numbers from the uniform distribution [a,b)
     *
     */
    float * readOutputs();
    /*
    * this function returns the Array of output values
     */


    /*
     * these are the variables for Backpropagation-Decorrelation learning
     */
    float * errors;
    float * oldErrors;
    float * oldOutputs;
    float * oldIntermediates;


    int EvaluationCollectionStep;

    /*
     * The following functions are called from within other functions
     * */
    void cullInnerVector(float threshold = 0.5);
    /*
     * limits (thresholded) the hidden neuron activation range between steps using an appropriate transfer function
     * */


    void cullOutput(float threshold = 0.5/*unused*/);
    /*
     * limits (thresholded) the output neuron activation range between steps using an appropriate transfer function
     */

    float sigmoid(float x);
    /*
     * approximated logistic function (fermi-dirac)
     */

    float deriSigmoid(float x);
    /*
     * derivative of the above function, used in backpropagation-decorrelation learning
     */

    double tanh(double x, bool flag);
    /*
     * tangens hyperbolicus function
     */

    double updateGauss_gain(double y, double eta);
    /*
     * Calculates the change in parameter b (gain) of a neuron using Gaussian IP learning rule
     * */

    void normalizeInnerWeights(float density);
    /*
     * This function scales the matrix of inner Weights, so that the highest eigenvalue is equal to "density".
     * Thus, the Echo-State-Property of the Network is ensured.
     *
     */

    void normalizeInputWeights(float density);
    /*
     * This function scales the matrix of start Weights (input to reservoir connections), so that the highest eigenvalue is equal to "density"
     */

      std::ofstream out;
      int inputNeurons;
      int outputNeurons;
      int networkNeurons;


      int stepsRun;
      // networkNeurons should be about 20-100 times bigger than inputs or outputs
      matrix::Matrix * inputs;      //the next three matrices hold neuron activations
      matrix::Matrix * outputs;
      matrix::Matrix * intermediates;

      matrix::Matrix * identity;
      matrix::Matrix * leak_mat;
      matrix::Matrix * inverse_leak_mat;

      //matrices needed for online learning. for details, please refer to sakyas paper
      matrix::Matrix * onlineLearningAutocorrelation;
      matrix::Matrix * onlineError;         //the error between input and desired output
      matrix::Matrix * transposedIntermediates; // intermediate for a multiplication step
      matrix::Matrix * toChangeOutputWeights;   //  the final change of output weights is onlineError times this

      matrix::Matrix * gainvector;

      //matrix for experimental learning
      matrix::Matrix * history;

      matrix::Matrix * noise;

      // the weights:


      /*startweights = new */matrix::Matrix * startweights;
      /*innerweights = new */matrix::Matrix * innerweights;
      /*endweights = new */matrix::Matrix * endweights;
                           matrix::Matrix * feedweights;

      bool throughputConnections; //whether inputs connect to outputs
      bool feedbackConnections;//whether outputs feed back to hidden layer
      bool leak; // enable or disable leaky integrated neurons
      bool RCneuronNoise;
      bool Loadweight;

      int withRL;

      /*Intrinsic plasticity parameters para_a and para_b*/

      double para_a;
      double para_b;
      bool  enable_IP;   // Enable or disable Gaussian IP

      double store_para_a[100000];
      double store_para_b[100000];
      double outputsCollection[10000];

      double leak_rate;
      double autocorr;
      int nonlinearity;
      int outnonlinearity;

      float InputWeightRange;
      float RCWeightRange;
      float FeedbackWeightRange;
      float NoiseRange;
      float input_scaling;

      int InputSparsity;
      int RCsparsity;
      int LearnMode;

};
#endif
