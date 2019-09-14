#include "EMGapproxESN.h"

EMGapproxESN::EMGapproxESN(
        string _inputPath,
        string _targetPath,
        string _resultPath,
        string _dir,
        unsigned int _num,
        unsigned int _numberOfInputs,
        unsigned int _numberOfOutputs,
        unsigned int _numberOfHiddenUnits,
        unsigned int _learningMode,
        unsigned int _internalNonlinearity,
        unsigned int _outputNonlinearity,
        double _inputSparsity,
        double _internalSparsity,
        double _learningRate,
        double _leak,
        double _percentage_1,
        double _percentage_2,
        unsigned int _repetition,
        unsigned int _iterationLimit,
        unsigned int _verboseLevel,
        char _delimiter,
        unsigned int _numOfHeaderLines ) {

    /// Create ESN input vector
    inputValues             = new float[numberOfInputs];
    /// Create ESN target output vector
    targetValues            = new float[numberOfOutputs];

    // Open file for saving results
    resultsFile.open( _resultPath, ios::out );

    /// Setting up ANN parameters
    numberOfInputs          = _numberOfInputs;
    numberOfOutputs         = _numberOfOutputs;
    numberOfHiddenUnits     = _numberOfHiddenUnits;

    learningMode            = _learningMode;
    internalNonlinearity    = _internalNonlinearity;
    outputNonlinearity      = _outputNonlinearity;

    inputSparsity           = _inputSparsity;
    internalSparsity        = _internalSparsity;

    learningRate            = _learningRate;
    leak                    = _leak;

    percentage_1            = _percentage_1;
    percentage_2            = _percentage_2;
    repetition              = _repetition;
    iterationLimit          = _iterationLimit;

    readData( _inputPath, _targetPath, _delimiter, _numOfHeaderLines );

    ESN                     = new ESNetwork(numberOfInputs/*no. input*/,numberOfOutputs /*no. output*/, numberOfHiddenUnits /*rc hidden neurons*/, false /*W_back, feedback from output to hiddens*/, false/*feeding input to output*/, leak /*leak = 0.0-1.0*/, false /*IP*/);
    ESN->outnonlinearity    = outputNonlinearity; //0 = linear, 1 = sigmoid (logistic), 2  = tanh: transfer function of an output neuron
    ESN->nonlinearity       = internalNonlinearity; //0 = linear, 1 = sigmoid (logistic), 2  = tanh: transfer function of all hidden neurons
    ESN->withRL             = 2; //2 = stand ESN learning, 1 = RL with TD learning

    ESN->InputSparsity      = inputSparsity; //if 0 = input connects to all hidden neurons, if 100 = input does not connect to hidden neurons
    ESN->autocorr           = pow(10,4);//set as high as possible, default = 1
    ESN->InputWeightRange   = 0.5; //Input scaling--> scaling of input to hidden neurons, default 0.15 means [-0.15, +0.15]
    ESN->LearnMode          = learningMode; //RLS = 1 (learning rate needs to be large, 0.99). LMS = 2 (learning rate needs to be very small, e.g., 0.01)
    ESN->Loadweight         = false; // true = loading learned weights
    ESN->NoiseRange         = 0.0001;// amplitude of noise
    ESN->RCneuronNoise      = true;//= constant fixed bias, true = changing noise bias every time

    ESN->generate_random_weights( internalSparsity/*if 10 means 10% sparsity = 90% connectivity */, 0.9 /*Spectral radius < 1.0 to maintain echo state property, 1.2-1.5 = chaotic*/);

    /// Reset input and target values
    resetInputValues();
    resetOutputValues();
    resetMSE();

    verboseLevel            = _verboseLevel;
    ESN->verboseLevel       = _verboseLevel;

    train();
    test( true );

    save( _num, _dir );

}

EMGapproxESN::EMGapproxESN(
        string _inputPath,
        string _targetPath,
        string _resultPath,
        string _dir,
        unsigned int _num,
        double _percentage_1,
        double _percentage_2,
        unsigned int _verboseLevel,
        char _delimiter,
        unsigned int _numOfHeaderLines ) {

    /// Load a network
    ESN                     = new ESNetwork( _num, _dir );

    /// Extracting ANN parameters
    numberOfInputs          = ESN->inputNeurons;
    numberOfHiddenUnits     = ESN->networkNeurons;
    numberOfOutputs         = ESN->outputNeurons;

    /// Create ESN input vector
    inputValues             = new float[numberOfInputs];
    /// Create ESN target output vector
    targetValues            = new float[numberOfOutputs];

    percentage_1            = _percentage_1;
    percentage_2            = _percentage_2;

    /// Reset input and target values
    resetInputValues();
    resetOutputValues();
    resetMSE();

    verboseLevel            = _verboseLevel;
    ESN->verboseLevel       = _verboseLevel;

    readData( _inputPath, _targetPath, _delimiter, _numOfHeaderLines );

    /// Open file for saving results
    resultsFile.open( _resultPath, ios::out );

    test( false );

    //    string greenBold = "\033[32;1m";
    //    string reset = "\033[0m";
    //    cout << "before" + greenBold + " green " + reset + "after" << endl;

}


EMGapproxESN::~EMGapproxESN(){

    delete[] ESN;
    delete[] inputValues;
    delete[] targetValues;

    resultsFile.close();

}

void EMGapproxESN::save( unsigned int num, string dir ) {

    ESN->writeParametersToFile( num, dir );
    ESN->writeStartweightsToFile( num, dir );
    ESN->writeInnerweightsToFile( num, dir );
    ESN->writeEndweightsToFile( num, dir );
    ESN->writeNoiseToFile( num, dir );

}

void EMGapproxESN::load( unsigned int num, string dir ) {

    ESN->readParametersFromFile( num, dir );
    ESN->readStartweightsFromFile( num, dir );
    ESN->readInnerweightsFromFile( num, dir );
    ESN->readEndweightsFromFile( num, dir );
    ESN->readNoiseFromFile( num, dir );

}

/**
 *
 *
 * If the data is organized in plain .txt files as follows:
 *     input1(t0)    input2(t0)    input3(t0)    ...
 *     input1(t1)    input2(t1)    input3(t1)    ...
 *          ...                 ...                 ...
 * where the separator is '\t', then it will be read to
 * inputData and targetData as:
 *     [
 *          [ input1(t0)    input2(t0)    input3(t0)    ... ],
 *          [ input1(t1)    input2(t1)    input3(t1)    ... ],
 *          ...
 *     ]
 * so that e.g.
 *     inputData[0][0] = input1(t0)
 *     inputData[0][1] = input2(t0)
 *     inputData[1][0] = input1(t1)
 *     ...
 *
 * @param pathToInput
 * @param pathToTarget
 * @param delimiter
 * @param numberOfHeaderLines
 */
void EMGapproxESN::readData(
        string pathToInput,
        string pathToTarget,
        char delimiter,
        unsigned int numberOfHeaderLines ) {

    if( verboseLevel >= 1 ) cout << "Reading data...";

    vector< vector< double > > inputData, targetData;

    DelimitedFileReader::read( pathToInput, delimiter, inputData, numberOfHeaderLines  );
    DelimitedFileReader::read( pathToTarget, delimiter, targetData, numberOfHeaderLines  );

    if( verboseLevel >= 1 ) cout << "\t\t\t\t[OK]" << endl;

//    // Testing
//    for( unsigned int i = 0; i < inputData.size(); i++ ) {
//
//        for( unsigned int j = 0; j < inputData.size(); j++ ) {
//
//            cout << "inputData[" << i << "][" << j << "]:" << inputData[i][j] << endl;
//
//        }
//
//    }

    /// Separating training data from testing data.
    separateData( inputData, targetData );

}

/**
 * Divides the data available into two batches. One for training and one for
 * testing the network
 *
 * trainingUnits and testingUnits will contain data following this pattern:
 *     trainingUnits[0][0][0] = input1(t0)
 *     trainingUnits[0][0][1] = input2(t0)
 *     trainingUnits[0][1][0] = target1(t0)
 *     trainingUnits[0][1][1] = target2(t0)
 *     trainingUnits[1][0][0] = input1(t1)
 *     trainingUnits[1][0][1] = input2(t1)
 *     ...
 *
 * @param inputData
 * @param targetData
 * @param percentage
 */
void EMGapproxESN::separateData(
        vector< vector< double > > & inputData,
        vector< vector< double > > & targetData ) {

    if( verboseLevel >= 1 ) cout << "Separating data...";

    // The number of iterations after which the learning units created are set aside for
    // cross evaluation and testing purposes
    unsigned int index_1 = ( percentage_1 / 100.0 ) * inputData.size();
    unsigned int index_2 = index_1 + ( percentage_2 / 100.0 ) * ( inputData.size() - index_1 );

    /// Even if the number of inputs and outputs are not the same,
    /// the time steps are equal thus the outer loop can be common
    for( unsigned int i = 0; i < inputData.size(); i++ ) {

        vector< double > inputs;
        for( unsigned int j = 0; j < numberOfInputs; j++ ) {

            inputs.push_back( inputData[i][j] );

        }

        vector< double > targets;
        for( unsigned int j = 0; j < numberOfOutputs; j++ ) {

            targets.push_back( targetData[i][j] );

        }

        vector< vector< double > > unit{ inputs, targets };




        if ( i <= index_1 )
          trainingUnits.push_back( unit );
        else if( i <= index_2 )
          validationUnits.push_back( unit );
        else
          testingUnits.push_back( unit );

    }

    if( verboseLevel >= 1 ) cout << "\t\t\t\t[OK]" << endl;

}

/**
 * Sets the values given as parameter to be the inputs of the ANN
 *
 * @param values
 */
void  EMGapproxESN::writeInput( float * values ) {

    for( unsigned int i = 0; i < numberOfInputs; i++ ) {

        inputValues[i] = values[i];

    }

}

/**
 * @return the latest output of the ANN
 */
float * EMGapproxESN::readOutput() const {

    float * temp = new float[numberOfOutputs];

    for( unsigned int i = 0; i < numberOfOutputs; i++ ) {

        temp[i] = targetValues[i];

    }

    return temp;

}

void EMGapproxESN::activate() {

    iteration++;

    ESN->setInput( inputValues, numberOfInputs/* no. input*/ ); // Call ESN

    // ESN Learning function
    ESN->takeStep( targetValues, learningRate /*0.9 RLS*/, 1 /*no td = 1 else td_error*/, learningFlag/* true= learn, false = not learning learn_critic*/, iteration/*0*/);

    // TODO temporary solution
    target_ESN = targetValues[0];
    output_ESN = ESN->outputs->val(0, 0); // Read out the output of ESN

    // ESN->printMatrix(ESN->endweights); //print weight matrix on screen

    /// Calculate online error at each time step
    squaredError = ( target_ESN - output_ESN ) * ( target_ESN - output_ESN );

    mse += squaredError;



    /// Testing
    if( verboseLevel >= 2 ) {

        cout << "Iteration:\t\t"                << iteration << endl;
        cout << "inputValues[0]:\t"             << inputValues[0] << endl;
        cout << "targetValues[0]:\t"            << targetValues[0] << endl;
        cout << "output_ESN:\t\t"               << output_ESN << endl;
        cout << "Online Training error:\t"      << squaredError << endl;

    }

    /// Save results if tests are being performed
   // if( !learningFlag ) {

     //   resultsFile << output_ESN << "\t" << target_ESN << "\t" << squaredError << "\n" << flush; //SAVE DATA

    //}

    resultsFile << output_ESN << "\t" << target_ESN << "\t" << squaredError << "\n" << flush; //SAVE DATA

}

void EMGapproxESN::train() {

    learningFlag    = true;
    iteration       = 0;

    if( verboseLevel >= 1 ) {

        cout << endl;
        cout << "#########################################################################" << endl;
        cout << "##                          Training started...                        ##" << endl;
        cout << "#########################################################################" << endl;

    }

    /// Repeat training as many times as specified or until iterationLimit is reached
    for( unsigned int r = 0; r < repetition; r++ ) {

        /// Iterate through the time steps...
        for( unsigned int i = 0; i < trainingUnits.size(); i++ ) {


            /// Escape condition
            if( iteration > iterationLimit ) break;

            /// ...set inputs...
            for( unsigned int j = 0; j < numberOfInputs; j++ ) {

                inputValues[j] = trainingUnits[i][0][j];

            }

            /// ...set outputs...
            for( unsigned int j = 0; j < numberOfOutputs; j++ ) {

                targetValues[j] = trainingUnits[i][1][j];

            }

            activate(  );

        }

        /// Escape condition
        if( iteration > iterationLimit ) break;

    }

    if( verboseLevel >= 1 ) {

        cout << "##                          Training finished.                         ##" << endl;
        cout << endl;

    }

}

void EMGapproxESN::test( bool isValidation ) {

    learningFlag    = false;
    resetMSE();

    if( verboseLevel >= 1 ) {

        cout << "#########################################################################" << endl;
        cout << "##                           Testing started...                        ##" << endl;
        cout << "#########################################################################" << endl;

    }

    /// Adding header
    // resultsFile << "Output" << "\t" << "Target" << "\t" << "Squared Error" << "\n" << flush;

    /// Iterate through the timesteps...

    for( unsigned int i = 0; i < validationUnits.size(); i++ ) {
    //for( unsigned int i = 0; i <  trainingUnits.size(); i++ ) {

    /// ...set inputs...
        for( unsigned int j = 0; j < numberOfInputs; j++ ) {

            if( isValidation ) {

                inputValues[j] = validationUnits[i][0][j];

            } else {

                inputValues[j] = testingUnits[i][0][j];

            }

        }

        /// ...set outputs...
        for( unsigned int j = 0; j < numberOfOutputs; j++ ) {

            if( isValidation ) {

                targetValues[j] = validationUnits[i][1][j];

            } else {

                targetValues[j] = testingUnits[i][1][j];

            }

        }

        activate();


    }

    /// Calculate MSE
    if( isValidation ) {

        mse = mse / validationUnits.size();

    } else {

        mse = mse / testingUnits.size();

    }


       resultsFile << "Output" << "\t" << "Target" << "\t" << "Squared Error" << "\n" << flush;
       /// Save MSE
       resultsFile << "\n" << "MSE" << "\t" << mse << "\n" << flush;
       //resultsFile << "\n" << "MSE" << "\t" << mse << "\n" << validationUnits.size() << "\n" << testingUnits.size()<<flush;

    if( verboseLevel >= 1 ) {

        cout << "##                           Testing finished.                         ##" << endl;
        cout << endl;

        cout << "MSE: " << mse << "Hidden unit: "<< numberOfHiddenUnits << "Iteration: "<< iterationLimit<< endl;
        cout << "Training data: " << trainingUnits.size() << "Validation data: "<< validationUnits.size() << "Total data: "<< trainingUnits.size()+validationUnits.size() << endl;

    }

}
