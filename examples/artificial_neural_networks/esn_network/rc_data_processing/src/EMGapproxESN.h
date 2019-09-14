#ifndef EMG_ESN_APPROX_RBF_H
#define EMG_ESN_APPROX_RBF_H

#include "DelimitedFileReader.h"

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
// #include <cstdlib>

#include "math.h"
#include <algorithm>    // max()
#include <stdio.h>        // NULL
#include <stdlib.h>       // srand, rand
#include <time.h>        // time

//#include "utils/esn-framework/networkmatrix.h" // XXX Include the networkmatrix last, otherwise it won't work!!!
#include "examples/artificial_neural_networks/esn_network/rc_data_processing/include/networkmatrix.h" // XXX Include the networkmatrix last, otherwise it won't work!!!



typedef vector< vector< vector< double > > > vector3Ddouble;
// using namespace std;

class EMGapproxESN{

public:

    EMGapproxESN(
        string,                             // Input path
        string,                             // Target path
        string = "results/results.txt",     // Result path
        string = "save_files/",             // Save directory
        unsigned int = 1,                   // Save file number
        unsigned int = 1,                   // # of input neurons
        unsigned int = 1,                   // # of output neurons
        unsigned int = 100,                  // # of hidden neurons
        unsigned int = 1,                   // Learning mode
        unsigned int = 2,                   // Internal nonlinearity
        unsigned int = 0,                   // Output nonlinearity
        double = 50.0,                      // Input sparsity
        double = 50.0,                      // Internal sparsity
        double = 0.998,                     // Learning rate / Forgetting factor
        double = 0.33,                      // Leakage
        double = 80.0,                      // Percentage 1
        double = 50.0,                      // Percentage 2
        unsigned int = 1,                   // Repetition
        unsigned int = 5000,                // Iteration limit
        unsigned int = 0,                   // Verbose level
        char = '\t',                        // Delimiter
        unsigned int = 0 );                 // Number of header lines

    EMGapproxESN(
        string,                             // Input path
        string,                             // Target path
        string = "results/results.txt",     // Result path
        string = "save_files/",             // Save directory
        unsigned int = 1,                   // Save file number
        double = 80.0,                      // Percentage 1
        double = 50.0,                      // Percentage 2
        unsigned int = 0,                   // Verbose level
        char = '\t',                        // Delimiter
        unsigned int = 0 );                 // Number of header lines

    ~EMGapproxESN();

    void train();
    void test( bool = true );

    void save( unsigned int = 1, string = "save_files/" );
    void load( unsigned int = 1, string = "save_files/" );

private:

    unsigned int numberOfInputs, numberOfOutputs, numberOfHiddenUnits;
    unsigned int learningMode, internalNonlinearity, outputNonlinearity, withRL, iteration;
    double leak, inputSparsity, internalSparsity, learningRate, error, percentage_1, percentage_2;
    bool learningFlag, verboseLevel;
    unsigned int repetition, iterationLimit;

    ESNetwork * ESN;
    float * inputValues;  // ESinput
    float * targetValues; // ESTrainOutput

    double target_ESN, input_ESN, output_ESN; // Temporary variables, used for testing

    double mse; // Mean Square Error
    double squaredError;

    vector3Ddouble trainingUnits, validationUnits, testingUnits;

    ofstream resultsFile; // File to save the output

    void readData( string, string, char = '\t', unsigned int = 0 );
    void separateData( vector< vector< double > >&, vector< vector< double > >& );
    void writeInput( float * );
    float * readOutput() const;
    inline void resetInputValues() { for( unsigned int i = 0; i < numberOfInputs; i++ ) { inputValues[i] = 0.0; } }
    inline void resetOutputValues() { for( unsigned int i = 0; i < numberOfOutputs; i++ ) { targetValues[i] = 0.0; } }
    inline void resetMSE() { mse = 0.0; }
    void activate();

};

#endif
