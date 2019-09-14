/**
 * networkmatrix.h
 *
 * Created on:          Mar 22, 2012
 * Last Modified on:    Feb 12, 2016
 *
 * Authors: Andrej fillipow and Sakyasingha Dasgupta
 * Extended by Timon Tomas (2016)
 *
 * Please refer to Readme for basic details
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

        /**
         * The following functions have to be called manually, and are necessary for functioning
         */

        /**
         * Main constructor
         *
         * Used for training
         *
         * Generate the ESN network with all internal parameters defined
         * Takes the number of input, output and hidden neurons, whether inputs should feed into outputs, whether outputs
         * should feed back, and how quickly neurons should leak, in that order. Remember to initialize the weights!
         *
         * @param a
         * @param b
         * @param c
         * @param param1
         * @param param2
         * @param param3
         * @param param4
         */
        ESNetwork( unsigned int a, unsigned int b, unsigned int c, bool param1 = false, bool param2 = false, double param3 = 0.3, bool param4 = false );

        /**
         * Main constructor
         *
         * Used for testing
         *
         * @param num
         * @param dir
         */
        ESNetwork( unsigned int num, std::string dir );

        /**
         * Default destructor - handle or dangling objects
         */
        ~ESNetwork();

        /**
         * Generating function for the input, reservoir and output weights. Sparsity governs percentage of matrix sparseness.
         * Inputweights drawn randomnly from the uniform distribution [-0.5,0.5). Innerweights drawn randomnly from the uniform
         * distribution [-1.1). Output weights are all initialised to 0. Innerweights are subsequently scaled to the specific
         * spectral_radius
         *
         * @param sparsity
         * @param spectral_radius
         */
        void generate_random_weights( int sparsity = 90 /*95*/, float spectral_radius = 0.95 /*0.95*/ );


        /**
         * Same as above except with full random Neighboured weighting
         *
         * @param sparsity
         * @param spectral_radius
         * @param nNextNeighbours
         */
        void generate_neighbour_weights( int sparsity = 90, float spectral_radius = 0.95, int nNextNeighbours = 15 );

        /**
         * Accepts an array of floats, of length size, and sets the inputs to these given values. can use resetInput()
         * or scrambleInput() instead
         *
         * @param Input
         * @param size
         */
        void setInput( float *Input, unsigned int size );

        /**
         * Offline training of the output weights. The net is trained over specific "time" steps, the first "discardedTimesteps" time steps are
         * not used to train. This serves as warm up for the ESN network.
         *
         * @param Inputs
         * @param Outputs
         * @param time
         * @param discardedTimesteps
         */
        void trainOutputs( float * Inputs, float * Outputs, int time, int discardedTimesteps = 30 );

        /**
         * Online training of the output weights using the Recursive least squares algorithm. For algorithm introduction
         * check http://en.wikipedia.org/wiki/Recursive_least_squares_filter
         *
         * @param Outputs
         * @param forgettingFactor
         * @param td_error
         * @param param
         */
        void trainOnlineRecursive( float * Outputs, float forgettingFactor, float td_error, double param = 0.0 );

        /**
         *
         * Least Means squared learning of Endweights
         *
         * @param Outputs
         * @param forgettingFactor
         * @param td_error
         * @param param
         */
        void trainOnlineLMS( float * Outputs, float forgettingFactor, float td_error, double param = 0.0 );

        /**
         * Backpropagation-Decorrelation learning function. This is intended for online learning purposes. Note that this is not fully functional yet.
         *
         * @param Outputs
         * @param learningRate
         */
        void trainBackpropagationDecorrelation( float * Outputs, float learningRate );

        /**
         * Called at every discrete time step, calculating the inner neuron activations and outputs for the next discrete timestep.
         * If "learn_online" is true, the ESN learns the outputs specified in
         * "Outputs" via "trainOnlineRecursive()"
         * NOTE: the learningRate parameter acts as the forgetting factor for the trainOnlineRecursive() function.
         *
         * @param Outputs
         * @param learningRate
         * @param td_error
         * @param learn
         * @param timestep
         * @param param
         */
        void takeStep( float * Outputs, float learningRate, float td_error, bool learn = true, int timestep = 1, double param = 0.0 );

        /**
         * The ESN takes a step without online learning, as above. this is called e.g. from "trainOutputs()"
         */
        void takeStep() {

            float nothing[0];
            takeStep( nothing, 1, 0, false );

        }

        /**
         * The following functions have to be called manually, and are optional
         */

        /**
         *
         * @param start
         * @param end
         * @param desiredOutputs
         * @return the mean square error between given Outputs and computed outputs over "time" discrete timesteps. Please refer to readme for further details.
         */
        float evaluatePerformance( int start,int end, float * desiredOutputs );

        /**
         * Writers
         */
        void writeParametersToFile( unsigned int num, std::string dir = "" );

        void writeStartweightsToFile( unsigned int num, std::string dir = "" );

        void writeInnerweightsToFile( unsigned int num, std::string dir = "" );

        void writeInneractivityToFile( unsigned int num, std::string dir = "" );

        /**
         * Write the Endweights (RC to output neurons) values to a text file
         *
         * @param num
         */
        void writeEndweightsToFile( unsigned int num, std::string dir = "" );

        void writeNoiseToFile( unsigned int num, std::string dir = "" );


        /**
         * Readers
         */
        void readParametersFromFile( unsigned int num, std::string dir = "" );

        void readStartweightsFromFile( unsigned int num, std::string dir = "" );

        void readInnerweightsFromFile( unsigned int num, std::string dir = "" );

        void readEndweightsFromFile( unsigned int num, std::string dir = "" );

        void readNoiseFromFile( unsigned int num, std::string dir = "" );

        /**
         * Resets input to zero
         */
        void resetInput();

        /**
         * Randomizes input
         */
        void scrambleInput();

        /**
         * Prints the passed matrix to the terminal
         *
         * @param printedMatrix
         */
        void printMatrix( matrix::Matrix *printedMatrix );

        /**
         * Prints the outputs to the terminal
         */
        void printOutputToTerminal();

        /**
         * Prints the activations of the inner neurons to the terminal
         */
        void printNeuronsToTerminal();

        /**
         * Generate numbers from the uniform distribution [a,b)
         *
         * @param a
         * @param b
         * @return
         */
        double uniform( double a, double b );

        /**
         * @return the Array of output values
         */
        float * readOutputs();

        /**
         * these are the variables for Backpropagation-Decorrelation learning
         */
        float * errors;
        float * oldErrors;
        float * oldOutputs;
        float * oldIntermediates;


        int EvaluationCollectionStep;

        /**
         * The following functions are called from within other functions
         */

        /**
         * Limits (thresholded) the hidden neuron activation range between steps using an appropriate transfer function
         *
         * @param threshold
         */
        void cullInnerVector( float threshold = 0.5 );

        /**
         * Limits (thresholded) the output neuron activation range between steps using an appropriate transfer function
         *
         * @param threshold
         */
        void cullOutput( float threshold = 0.5/*unused*/ );

        /**
         * Approximated logistic function (Fermi-Dirac)
         *
         * @param x
         * @return
         */
        float sigmoid( float x );

        /**
         * Derivative of the above function, used in backpropagation-decorrelation learning
         *
         * @param x
         * @return
         */
        float deriSigmoid( float x );

        /**
         * Tangens hyperbolicus function
         *
         * @param x
         * @param flag
         * @return
         */
        double tanh( double x, bool flag );

        /**
         * Calculates the change in parameter b (gain) of a neuron using Gaussian IP learning rule
         *
         * @param y
         * @param eta
         * @return
         */
        double updateGauss_gain( double y, double eta );

        /**
         * Scales the matrix of inner Weights, so that the highest eigenvalue is equal to "density".
         * Thus, the Echo-State-Property of the Network is ensured.
         *
         * @param density
         */
        void normalizeInnerWeights( float density );

        /**
         * Scales the matrix of start Weights (input to reservoir connections), so that the highest eigenvalue is equal to "density"
         *
         * @param density
         */
        void normalizeInputWeights( float density );

        std::ofstream out;
        unsigned int inputNeurons;
        unsigned int outputNeurons;
        unsigned int networkNeurons;


        int stepsRun;
        /// networkNeurons should be about 20-100 times bigger than inputs or outputs
        matrix::Matrix * inputs;        //the next three matrices hold neuron activations
        matrix::Matrix * outputs;
        matrix::Matrix * intermediates;

        matrix::Matrix * identity;
        matrix::Matrix * leak_mat;
        matrix::Matrix * inverse_leak_mat;

        /// Matrices needed for online learning. for details, please refer to sakyas paper
        matrix::Matrix * onlineLearningAutocorrelation;
        matrix::Matrix * onlineError;                   //the error between input and desired output
        matrix::Matrix * transposedIntermediates;       // intermediate for a multiplication step
        matrix::Matrix * toChangeOutputWeights;         //  the final change of output weights is onlineError times this

        matrix::Matrix * gainvector;

        /// matrix for experimental learning
        matrix::Matrix * history;

        matrix::Matrix * noise;

        /// the weights:
        /*startweights = new */matrix::Matrix * startweights;
        /*innerweights = new */matrix::Matrix * innerweights;
        /*endweights = new */matrix::Matrix * endweights;
        matrix::Matrix * feedweights;

        bool throughputConnections; // Whether inputs connect to outputs
        bool feedbackConnections;   // Whether outputs feed back to hidden layer
        bool leak;                  // Enable or disable leaky integrated neurons
        bool RCneuronNoise;
        bool Loadweight;

        int withRL;

        /// Intrinsic plasticity parameters para_a and para_b
        double para_a;
        double para_b;
        bool enable_IP;   // Enable or disable Gaussian IP

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

        unsigned int verboseLevel;

};
#endif
