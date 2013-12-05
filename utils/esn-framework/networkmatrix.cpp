/*  networkmatrix.cpp
 *
 *  Contains the ESNetwork class member function definitions
 *
 *  Created on: Mar 22, 2012
 *  modified on: Aug 13th, 2013
 *  Author: Andrej Fillipow and Sakyasingha Dasgupta
 */
#include <fstream>
#include <string.h>
#include "networkmatrix.h"
#include <sstream>



std::ofstream out1; //the program prints inputs and outputs into the file "output", and  outputs into terminal
std::ofstream out2;
std::ofstream out3;
std::ofstream out4;
std::ofstream out5;


//using namespace std;

ESNetwork::ESNetwork(int a, int b, int c , bool param1, bool param2, double param3, bool param4)
{
	RCneuronNoise = false;
	Loadweight == false;
	enable_IP = param4;

	autocorr = 1; //.0;

	LearnMode == 1 ;// RLS Learning default, choose '2' for LMS learning

	nonlinearity = 2;        // RC neuron nonlinearity- ('0'- linear, '1'-sigmoid, '2'-tanh(default))
	outnonlinearity = 2;     // output neuron nonlinearity- ('0'- linear, '1'-sigmoid, '2'-tanh(default))

	//parameters:
	throughputConnections = param1; // input to output direct connection
	feedbackConnections = param2; //output to RC connections

	stepsRun = 0;    //Washout - run RC for some timesteps initially

	// turn on RLS learning for combinatorial learning
	withRL=1; // change to '2' for standard ESN

	leak_rate = param3; // rate of leakage of neurons

	if (leak_rate ==0)
		leak = false;
	else
		leak = true;
	// enables or disables leaky integrated neurons

	//numbers of neurons used in the net
	// networkNeurons should be about 20-100 times bigger than inputs or outputs

	inputNeurons = a;
	outputNeurons = b;
	networkNeurons = c;

	//the actual matrices that hold the output of the neurons
	inputs = new matrix::Matrix(inputNeurons,1);
	outputs = new matrix::Matrix(outputNeurons,1);
	intermediates = new matrix::Matrix(networkNeurons,1);
	leak_mat = new matrix::Matrix(networkNeurons,networkNeurons);
	inverse_leak_mat = new matrix::Matrix(networkNeurons, networkNeurons);

	toChangeOutputWeights = new matrix::Matrix(networkNeurons,1);

   //gainvector = new matrix::Matrix(networkNeurons, 1);



	//Initializing leak matrix
	for (int i =0;i<networkNeurons;i++)
		for (int j=0;j<networkNeurons;j++)
			if (i==j) leak_mat->val(i,j) = leak_rate;
			else leak_mat->val(i,j)=0.0;

	for (int i =0;i<networkNeurons;i++)
			for (int j=0;j<networkNeurons;j++)
				if (i==j) inverse_leak_mat->val(i,j) = 1-leak_rate;
				else leak_mat->val(i,j)=0.0;

	//**********Weight matrices for the ESN framework****************//

	//input weight matrix
	startweights = new matrix::Matrix(networkNeurons, inputNeurons);

	feedweights = new matrix::Matrix(networkNeurons, outputNeurons);

	//Reservoir weight matrix. Matrix is bigger if the output feeds back
	int temp =  networkNeurons ;//+ (feedbackConnections ?  outputNeurons : 0);
	innerweights = new matrix::Matrix(temp, temp);

	//Output weight matrix. Matrix is bigger, if the output feeds back
	//temp = (throughputConnections?  inputNeurons : 0)+ networkNeurons;
	endweights   = new matrix::Matrix(outputNeurons, networkNeurons/*temp*/);

	//declare noise matrix
	noise = new matrix::Matrix(networkNeurons,1);

	//Matrix for online recursive learning
	onlineLearningAutocorrelation = new  matrix::Matrix(networkNeurons, networkNeurons);

	unsigned int i,j = 0; // initialise the function to a positive number

	//Initialization of RLS autocorrelation matrix
	for(i = 0; i < networkNeurons; i++)
		for(j=0;j< networkNeurons; j++)
		{
			if(i==j)
				onlineLearningAutocorrelation->val(i,j)= autocorr; //pow(10,5); // Need to set to a high value depending on task at hand. In general for artificial data set to high and for NIMM4 leave as it is
			else
				onlineLearningAutocorrelation->val(i,j)= 0.0;
		}


	errors = new float[outputNeurons];
	oldErrors = new float[outputNeurons];
	oldOutputs = new float[outputNeurons];
	oldIntermediates = new float[networkNeurons];
	history = new matrix::Matrix();
	*history = *endweights ;

	// for evaluation purposes:
	EvaluationCollectionStep = 0;
	//     outputsCollection = NULL;

	for (int i = 0; i< networkNeurons;i++)
	{
		oldIntermediates[i] =  0;
	}
	for (int i = 0; i< outputNeurons;i++)
	{
		errors[i] = oldErrors[i] = 0;
		oldOutputs[i] = 0;
	}

	for (int i = 0; i< 100000;i++)
	{
		store_para_a[i] =  0.0;
		store_para_b[i] = 0.0;
	}

	for (int i = 0; i< 10000;i++)
		outputsCollection[i]=0.0;

	// All array initializations complete
	std::cout<<"Arrays initialised" << "\n";
	out.open("output_matrix");

	/*-------------------- Weight Parameters -------------------------------*/

		 InputWeightRange = 0.15;
		 RCWeightRange = 1.0;
		 FeedbackWeightRange = 0.5;
		 NoiseRange = 0.001;

		 InputSparsity = 0; //50
		 RCsparsity = 90;

		 input_scaling = 0.5;

		/*-------------------- Weight Parameters ------------------------------*/


}

ESNetwork::~ESNetwork()
{
	delete inputs;
	delete outputs;
	delete intermediates;
	delete startweights;
	delete innerweights;
	delete endweights;
	delete gainvector;
	delete onlineLearningAutocorrelation;
	delete errors;
	delete oldErrors;
	delete oldOutputs;
	delete oldIntermediates;
	delete history;
	delete outputsCollection;
	out.close();
}


//Generate connection weights
void ESNetwork::generate_random_weights(int sparsity, float spectral_radius) //sparsity from 1-100, density 0-1, all connections random
{
	RCsparsity = sparsity;

	std::cout << "generating full-random weighting. \n \n";
	srand(time(NULL));

	unsigned int i,j = 0;

	if (InputSparsity == 0)
	{
		for(i = 0; i < startweights->getM(); i++)
		for (j=0; j< startweights->getN();j++)
		{
			startweights->val(i,j)= ESNetwork::uniform(-InputWeightRange,InputWeightRange);

		}

	}

	else {

		for(i = 0; i < startweights->getM(); i++)
				for (j=0; j< startweights->getN();j++)
				{
					if ((rand()%100) >= InputSparsity)
					{startweights->val(i,j)= ESNetwork::uniform(-InputWeightRange,InputWeightRange);}
					else
						startweights->val(i,j)= 0.0;

				}

	}

 //Feedback connections to all inner neurons
	for(i = 0; i < feedweights->getM(); i++)
			for (j=0; j< feedweights->getN();j++)
			{
				{feedweights->val(i,j)= ESNetwork::uniform(-FeedbackWeightRange,FeedbackWeightRange);}

			}

	//initialise the Reservoir weights
	j = 0;
	for(i = 0; i < innerweights->getM(); i++)
		for(j = 0; j < innerweights->getN(); j++)
		{
			if ((rand()%100) >= RCsparsity) { innerweights->val(i,j)= ESNetwork::uniform(-RCWeightRange,RCWeightRange);}
			else
				innerweights->val(i,j)= 0.000;
		}

	//Initialise the output weights
	j = 0;
	for(i = 0; i < endweights->getM(); i++)
		for(j = 0; j < endweights->getN(); j++)
			{ endweights->val(i,j)=0.000;}


	//initialize noise matrix
	for(i = 0; i < networkNeurons; i++)
		for(j = 0; j < 1; j++)
			noise->val(i,j) = ESNetwork::uniform(-NoiseRange,NoiseRange);

	ESNetwork::normalizeInnerWeights(spectral_radius);

	//ESNetwork::normalizeInputWeights(input_scaling);

	std::cout <<" finished \n";
}

void ESNetwork::generate_neighbour_weights(int sparsity, float spectral_radius, int nNextNeighbours) //like above, only connections to the nNextNeighbours neurons are made
{
	// Unused;
}

void ESNetwork::setInput(float *Input, int size)// sets the input neurons, called by trainOutputs() or manually from main.cpp
{
	for (int i = 0; i < size; i++)
	{
		inputs->val(i,0) = Input[i];
		//  printMatrix(inputs);
	}
}

void ESNetwork::trainOutputs(float * Inputs, float * Outputs, int time, int discardedTimesteps)
{
//*********************************************************************************************************
//	// Off-line batch mode learning not used.  USE RLS online learning instead (trainOnlineRecurssive function)
// ********************************************************************************************************
//	matrix::Matrix * stateMatrix = new
//			matrix::Matrix(time-discardedTimesteps, networkNeurons
//			/*+(feedbackConnections?outputNeurons:0)*/); //stateMatrix is the matrix ofstates
//	//std::cout << "initiated Training \n \n";
//
//	for(int i = 0; i < time; i++) //record states into state matrix
//	{
//		ESNetwork::setInput(Inputs+(i*inputNeurons), inputNeurons);
//		for (unsigned int j = 0; j < outputNeurons;j++)
//		{
//			outputs->val(j,0) = Outputs[i*outputNeurons+j];
//		}
//
//
//		ESNetwork::takeStep();
//		if (i >= discardedTimesteps)
//		{
//			int j = 0;
//
//			/*for(; j <  inputNeurons; j++)
//              {
//                stateMatrix->val(i-discardedTimesteps,j)= inputs->val(j,0);
//              }
//			 */
//			for(; j <  networkNeurons; j++)
//			{
//				stateMatrix->val(i-discardedTimesteps,j)=
//						intermediates->val(j,0)+ESNetwork::uniform(-0.05,0.05);
//			}
//			/*if (feedbackConnections)  //this part is tricky: if the outputs
//    feed back, they must be part of the state matrix
//              {
//                for  (; j <  networkNeurons+outputNeurons; j++)
//                {
//                  stateMatrix->val(i-discardedTimesteps,j)=
//    outputs->val(j-networkNeurons,0);
//                }
//              }*/
//		}
//	}
//	std::cout << "states found \n";
//	std::cout << "states matrix is: " << stateMatrix->getM() <<" times " <<
//			stateMatrix->getN() << std::endl;
//	/*
//	 *Here, the teacher outputs are collected
//	 *
//	 */
//	matrix::Matrix * desiredOutput = new
//			matrix::Matrix(time-discardedTimesteps, outputNeurons);
//	for(int i = discardedTimesteps; i < time; i++)
//	{
//		for(int j = 0; j < outputNeurons; j++)
//		{
//			desiredOutput->val(i-discardedTimesteps,j) =
//					Outputs[i*outputNeurons+j];
//		}
//	}
//	std::cout<<"desired outputs found \n";
//
//	/* generating: (s?¹ s)?¹ s^t D
//	 *
//	 */
//
//	matrix::Matrix * temp2 = new matrix::Matrix(inputNeurons +
//			networkNeurons, time-discardedTimesteps);
//	*temp2 = *stateMatrix;
//	// here was the mistake:
//	//*temp2 = temp2->pseudoInverse();
//	*temp2 = temp2->toExp(0xFF);
//	matrix::Matrix * temp3 = new matrix::Matrix(time-discardedTimesteps,
//			time-discardedTimesteps);
//	temp3->mult(*temp2, *stateMatrix);               // is (s' s) now
//
//
//
//	/*
//          matrix::Matrix * Eigens = new matrix::Matrix;
//	 *Eigens =  eigenValuesRealSym(*temp3);
//          temp3->mult(*temp3, (1/Eigens->val(0,0))*20);
//          delete Eigens;
//	 */
//
//
//	temp3->pseudoInverse();                           // is (s' s)?¹ now WARNING takes long time to compute
//	std::cout << "pseudo-Inverse is: " << temp3->getM() <<" times " <<
//			temp3->getN() << std::endl;
//
//	matrix::Matrix * temp4 = new matrix::Matrix(time-discardedTimesteps,
//			inputNeurons + networkNeurons);
//	*temp4 = *stateMatrix;
//	temp4->toExp(0xFF);                               //is s' now
//	temp4->mult(*temp4, *desiredOutput);              //is s' * D now
//	//temp4->mult(*temp4, *temp3);                      //is (s' s)?¹ s' now
//
//
//	std::cout << " s^T * D is "<<temp4->getM() <<" times "<<temp4->getN()<< std::endl;
//
//	temp4->mult(*temp3, *temp4);
//	std::cout << "(s?¹ s)?¹ s^t D is: " << temp4->getM() <<" times " <<
//			temp4->getN() << std::endl;
//	*endweights = *temp4;
//
//	/*
//          //alternate method:
//          matrix::Matrix * temp2 = new matrix::Matrix(inputNeurons +
//    networkNeurons, time-discardedTimesteps);
//	 *temp2 = *stateMatrix;
//	 *temp2 = temp2->pseudoInverse();
//
//
//          endweights->mult(*temp2, *desiredOutput);*/
//
//	endweights->toExp(0xFF);
//	std::cout << "endweights is: " << endweights->getM() <<" times " <<
//			endweights->getN() << std::endl;
//
//	delete stateMatrix;
//	delete temp2;
//	//delete temp3;
//	//delete temp4;
//	delete desiredOutput;

}

void ESNetwork::trainOnlineLMS(float * Outputs, float forgettingFactor, float td_error, double param)
{
	matrix::Matrix *temp = new matrix::Matrix(1, networkNeurons);
		matrix::Matrix *temp2 = new matrix::Matrix(1, networkNeurons);

		//the Error vector between input and output
		onlineError = new matrix::Matrix(1, outputNeurons);

		transposedIntermediates = new matrix::Matrix(1, networkNeurons);
		toChangeOutputWeights = new matrix::Matrix(endweights->getM(), endweights->getN());
		/*
		 * Step 1: calculate the transpose of the matrix of inner neuron states and the error between Outputs and training outputs
		 *
		 */

		*transposedIntermediates = *intermediates;
		transposedIntermediates->toTranspose();
		*temp = *endweights;
		temp->toTranspose();

		temp2->mult(*transposedIntermediates, *temp);

		switch (withRL){/* Set to td_error for actor-critic learning and otherwise to default */
		case 1 :
		{
			for (int i = 0; i < outputNeurons; i++)
			{
				onlineError->val(0,i) = td_error;//(atan(Outputs[i])-temp2->val(0,i));


			}
			break;
		}
		case 2:
		{
			for (int i = 0; i < outputNeurons; i++)
			{
				if (outnonlinearity == 2)
				onlineError->val(0,i) = (atan(Outputs[i])-temp2->val(0,i)); //compute the Error between output and desired output -- end STEP 1

				else if(outnonlinearity == 0 || outnonlinearity == 1)
					onlineError->val(0,i) = ((Outputs[i])-temp2->val(0,i));

			}
			break;
		}

		}

		onlineError->toTranspose();

		 temp->mult(*onlineError,*transposedIntermediates);

				   temp->mult(*temp,forgettingFactor);

			    if (param)
			    	temp->mult(*temp, param);

			    *endweights = *endweights + *temp;

			        delete temp;
			    	delete temp2;
			    	delete onlineError;
			    	delete transposedIntermediates;
			    	delete toChangeOutputWeights;

}



void ESNetwork::trainOnlineRecursive(float * Outputs, float forgettingFactor, float td_error, double param)
{

	matrix::Matrix *temp = new matrix::Matrix(1, networkNeurons);
	matrix::Matrix *temp2 = new matrix::Matrix(1, networkNeurons);

	//the Error vector between input and output
	onlineError = new matrix::Matrix(1, outputNeurons);

	transposedIntermediates = new matrix::Matrix(1, networkNeurons);
//	toChangeOutputWeights = new matrix::Matrix(networkNeurons,1); //new matrix::Matrix(endweights->getM(), endweights->getN());
	/*
	 * Step 1: calculate the transpose of the matrix of inner neuron states and the error between Outputs and training outputs
	 *
	 */

	*transposedIntermediates = *intermediates;
	transposedIntermediates->toTranspose();
	*temp = *endweights;
	temp->toTranspose();

	temp2->mult(*transposedIntermediates, *temp);



	switch (withRL){/* Set to td_error for actor-critic learning and otherwise to default */
	case 1 :
	{
		for (int i = 0; i < outputNeurons; i++)
		{
			onlineError->val(0,i) = td_error;//(atan(Outputs[i])-temp2->val(0,i));


		}
		break;
	}
	case 2:
	{
		for (int i = 0; i < outputNeurons; i++)
		{
			if (outnonlinearity == 2)
			onlineError->val(0,i) = (atan(Outputs[i])-temp2->val(0,i)); //compute the Error between output and desired output -- end STEP 1

			else if(outnonlinearity == 0 || outnonlinearity == 1)
				onlineError->val(0,i) = ((Outputs[i])-temp2->val(0,i));


		}
		break;
	}

	}


	/*
	 *  Step 2:Output weights change delta W_out is set to   onlineLearningAutocorrelation * x(t) / (forgetting factor +  X(t)^t * onlineLearningAutocorrelation* x(t) )
	 */
	toChangeOutputWeights->mult(*onlineLearningAutocorrelation, *intermediates);
	temp ->mult( *transposedIntermediates, *onlineLearningAutocorrelation);
	temp2->mult(*temp,*intermediates);

	double scale = 1/(forgettingFactor + temp2->val(0,0));
	toChangeOutputWeights->mult(*toChangeOutputWeights, scale); //end STEP 2


	/*
	 * Step 3: the onlineLearningAutocorrelation is diminished by  delta_w * x(t)^T *itself and divided by the forgetting factor
	 */

	temp->mult(*toChangeOutputWeights, *transposedIntermediates);
	temp2->mult(*temp, *onlineLearningAutocorrelation);

	*temp = *onlineLearningAutocorrelation - *temp2;
	onlineLearningAutocorrelation->mult(*temp, 1/forgettingFactor); //end STEP 3


	/*
	 * Finally, the weights are updated and temporary matrices are deleted
	 */
	toChangeOutputWeights->toTranspose();

	onlineError->toTranspose();

	temp->mult(*onlineError,*toChangeOutputWeights);

    endweights->add(*endweights, *temp);

	//*endweights = *endweights + *temp  ;
//	*endweights = *endweights + *outputnoise;

	//std::cout << "online learning step finished \n \n \n";

	delete temp;
	delete temp2;
	delete onlineError;
	delete transposedIntermediates;
//	delete toChangeOutputWeights;




}


void ESNetwork::trainBackpropagationDecorrelation(float * Outputs, float learningRate)
{
	// unused
}


//computes the new state of all neurons except input
void ESNetwork::takeStep(float * Outputs, float learningRate, float td_error , bool learn, int timestep, double param)
{

	matrix::Matrix * temp = new matrix::Matrix(1,networkNeurons);
	matrix::Matrix * feed = new matrix::Matrix(1,networkNeurons);
	matrix::Matrix * temp2 = new matrix::Matrix(1,networkNeurons);
	matrix::Matrix * temp3 = new matrix::Matrix(1,inputNeurons+networkNeurons);
	matrix::Matrix * temp4 = new matrix::Matrix(networkNeurons, networkNeurons);
	matrix::Matrix * temp5 = new matrix::Matrix(1,networkNeurons);
	matrix::Matrix * old_intermediates = new matrix::Matrix(networkNeurons,1);
	matrix::Matrix * temp6 = new matrix::Matrix(networkNeurons,1);
	matrix::Matrix * transposeoutputs = new matrix::Matrix(1,networkNeurons);


	if(leak== false)
	{
		temp->mult(*startweights, *inputs);

		if (feedbackConnections)
		{
			//temp2->mult(*innerweights, intermediates->above(*outputs))  ;
			//temp2->removeRows(networkNeurons);

			feed->mult(*feedweights,*outputs);
			temp2->mult( *innerweights, *intermediates);
		}
		else
		{
			temp2->mult( *innerweights, *intermediates);
		}
		if(!feedbackConnections) intermediates->add(*temp, *temp2);

		else if(feedbackConnections)
		{
			intermediates->add(*temp, *temp2);
     	intermediates->add(*intermediates,*feed);
		}
		//std::cout <<  "inner Neurons computed:\n";

		// For noise to every neuron. If set to False, only constant bias noise will be added. Default is False
     if(RCneuronNoise == true)
     {


				 for(int i = 0; i < networkNeurons; i++)
		    	                  	  for(int j = 0; j < 1; j++)
		    	                    noise->val(i,j) = ESNetwork::uniform(-NoiseRange,NoiseRange);

     }
		//Add noise to the inner reservoir

		intermediates->add(*intermediates, *noise);

		ESNetwork::cullInnerVector(0.6);

		if (throughputConnections)
		{
			//*temp3 = *inputs;
			*temp3 = *intermediates;
		//	*temp3 = temp3->beside(*intermediates);
			*temp3 = temp3->beside(*inputs);


		}
		else
		{
			*temp3 = *intermediates;
		}

	}
	if (leak == true)
	{

        old_intermediates->mult(*inverse_leak_mat, *intermediates);



		temp->mult(*startweights, *inputs);

	//	*transposeoutputs = *outputs;
	//	transposeoutputs->toTranspose();

		if (feedbackConnections)
		{     temp2->mult(*innerweights, intermediates->above(*outputs))  ;
		      temp2->removeRows(networkNeurons);

		}
		else
		{
			temp2->mult( *innerweights, *intermediates /*old_intermediates*/ /**intermediates*/);
		}


		intermediates->add(*temp, *temp2);



		if(RCneuronNoise == true)
		     {
               for(int i = 0; i < networkNeurons; i++)
				 for(int j = 0; j < 1; j++)
				  noise->val(i,j) = ESNetwork::uniform(-NoiseRange,NoiseRange);

		     }
				// adding bias to reservoir neurons
					        intermediates->add(*intermediates, *noise);

		ESNetwork::cullInnerVector(0.6);



		temp5->mult(*leak_mat, *intermediates);




		intermediates->add(*old_intermediates, *temp5);
		//std::cout <<  "inner Neurons computed:\n";


		if (throughputConnections)
		{
			*temp3 = *inputs;
			*temp3 = temp3->beside(*intermediates);
		}
		else
		{
			*temp3 = *intermediates;
		}




	}


	if (learn)
	{
		if (td_error!= 0)
		{
           if (LearnMode == 1) //Chooses whether RLS or LMS learning, default is 1-RLS learning
			 {trainOnlineRecursive(Outputs, learningRate, (td_error), param);

			  outputs->mult(*endweights, *temp3);
			 }

           else if (LearnMode == 2) //LMS
           {
        	   trainOnlineLMS(Outputs, learningRate, (td_error), param);

        	   outputs->mult(*endweights, *temp3);
           }

			//       outputsCollection[timestep] = outputs->val(0,0);

			    //     printMatrix(endweights);

		}
	}

	else {
		   if(Loadweight == false)
		    outputs->mult(*endweights, *temp3);

		   else if (Loadweight == true)
		   {
			//   outputs->mult(*storedweights, *temp3);
		   }

	}



	ESNetwork::cullOutput(0.6);

    delete feed;
	delete temp;
	delete temp2;
	delete temp3;
	delete temp4;
	delete temp5;
	delete temp6;
	delete old_intermediates;
	delete transposeoutputs;
}

float ESNetwork::evaluatePerformance(int start,int end, float * desiredOutputs)
{
	std::cout << "Evaluating Performance" <<std::endl;

	out1.open("output_error");
	int i = 0;
	double tot = 0.0;
	double error = 0;
	for(i = start; i < end; i++)
	{

		tot= (outputsCollection[i]-desiredOutputs[i])*(outputsCollection[i]-desiredOutputs[i]);
		//std::cout<<"error:"<<tot<<std::endl;
		out1<<tot<<"\n";

		error += tot;
	}
	out1.close();
	return error/(end-start);
}

void ESNetwork::writeInnerweightsToFile(int num)
{
	std::ostringstream fileNameStream("");
		fileNameStream<<"inner_weights_"<<num<<".txt";

		std::string fileName = fileNameStream.str();

		out4.open(fileName.c_str());

		for(int i = 0; i < this->innerweights->getM(); i++)
				{for(int j = 0; j < this->innerweights->getN(); j++)
					out4<<this->innerweights->val(i,j)<<" ";
	             out4<<"\n";

				}
		out4.close();
}

void ESNetwork::writeStartweightsToFile(int num)
{
	std::ostringstream fileNameStream("");
		fileNameStream<<"start_weights_"<<num<<".txt";

		std::string fileName = fileNameStream.str();

		out3.open(fileName.c_str());

		for(int i = 0; i < this->startweights->getM(); i++)
				{for(int j = 0; j < this->startweights->getN(); j++)
					out3<<this->startweights->val(i,j)<<" ";
	             out3<<"\n";

				}
		out3.close();
}

void ESNetwork::writeNoiseToFile(int num)
{
	std::ostringstream fileNameStream("");
		fileNameStream<<"noise_"<<num<<".txt";

		std::string fileName = fileNameStream.str();

		out5.open(fileName.c_str());

		for(int i = 0; i < this->noise->getM(); i++)
				{for(int j = 0; j < this->noise->getN(); j++)
					out5<<this->noise->val(i,j)<<" ";
	             out5<<"\n";

				}
		out5.close();
}


void ESNetwork::writeEndweightsToFile(int num)
{
	std::ostringstream fileNameStream("");
	fileNameStream<<"output_weights_"<<num<<".txt";

	std::string fileName = fileNameStream.str();

	out2.open(fileName.c_str());

	for(int i = 0; i < this->endweights->getM(); i++)
			{for(int j = 0; j < this->endweights->getN(); j++)
				out2<<this->endweights->val(i,j)<<" ";
             out2<<"\n";

			}
	out2.close();

}

void ESNetwork::writeInneractivityToFile(int num)
{
	std::ostringstream fileNameStream("");
		fileNameStream<<"inner_activity_"<<num<<".txt";

		std::string fileName = fileNameStream.str();

		out2.open(fileName.c_str());

		for(int i = 0; i < this->intermediates->getM(); i++)
				{for(int j = 0; j < this->intermediates->getN(); j++)
					out2<<this->intermediates->val(i,j)<<" ";
	             out2<<"\n";

				}
		out2.close();

}

void ESNetwork::readInnerweightsFromFile(int num)
{
	matrix::Matrix * temp = new matrix::Matrix(innerweights->getM(), innerweights->getN());

	char str[10];

	//Opens for reading the file

	std::ostringstream fileNameStream("");
	fileNameStream<<"inner_weights_"<<num<<".txt";

	std::string fileName = fileNameStream.str();

	std::ifstream file(fileName.c_str());


	for(int i = 0; i < innerweights->getM(); i++)
	{
	   for(int j = 0; j < innerweights->getN(); j++)
	   {
	         if ( !(file >> str) )
	         {
	             std::cerr << "error while reading file";
	             break;
	         }

	         if (str ==" ") break;

	       	 temp->val(i,j) = atof(str);
	   }
	   if ( !file ) break;
	}



	//this->innerweights = *temp;

	for(int i = 0; i < innerweights->getM(); i++)
		{
		   for(int j = 0; j < innerweights->getN(); j++)

             	this->innerweights->val(i,j) = (temp->val(i,j));
		}

	delete temp;



}

void ESNetwork::readStartweightsFromFile(int num)
{
	matrix::Matrix * temp = new matrix::Matrix(startweights->getM(), startweights->getN());

	char str[10];

	//Opens for reading the file

	std::ostringstream fileNameStream("");
	fileNameStream<<"start_weights_"<<num<<".txt";

	std::string fileName = fileNameStream.str();

	std::ifstream file(fileName.c_str());


	for(int i = 0; i < startweights->getM(); i++)
	{
	   for(int j = 0; j < startweights->getN(); j++)
	   {
	         if ( !(file >> str))
	         {
	             std::cerr << "error while reading start weights file";
	             break;
	         }

	         if (str ==" ") break;

	         temp->val(i,j) = atof(str);
	   }
	   if ( !file ) break;
	}

	//this->startweights = temp;
	for(int i = 0; i < startweights->getM(); i++)
			{
			   for(int j = 0; j < startweights->getN(); j++)

	             	this->startweights->val(i,j) = (temp->val(i,j));
			}
	delete temp;



}



void ESNetwork::readNoiseFromFile(int num)
{
	matrix::Matrix * temp = new matrix::Matrix(noise->getM(), noise->getN());

	char str[10];

	//Opens for reading the file

	std::ostringstream fileNameStream("");
	fileNameStream<<"noise_"<<num<<".txt";

	std::string fileName = fileNameStream.str();

	std::ifstream file(fileName.c_str());


	for(int i = 0; i < noise->getM(); i++)
	{
	   for(int j = 0; j < noise->getN(); j++)
	   {
	         if ( !(file >> str))
	         {
	             std::cerr << "error while reading start weights file";
	             break;
	         }

	         if (str ==" ") break;

	         temp->val(i,j) = atof(str);
	   }
	   if ( !file ) break;
	}

	//this->*noise = *temp;

	for(int i = 0; i < noise->getM(); i++)
			{
			   for(int j = 0; j < noise->getN(); j++)

	             	this->noise->val(i,j) = (temp->val(i,j));
			}

	delete temp;



}

void ESNetwork::readEndweightsFromFile(int num)
{
	matrix::Matrix * temp = new matrix::Matrix(endweights->getM(), endweights->getN());

		char str[10];

		//Opens for reading the file

		std::ostringstream fileNameStream("");
		fileNameStream<<"output_weights_"<<num<<".txt";

		std::string fileName = fileNameStream.str();

		std::ifstream file(fileName.c_str());


		for(int i = 0; i < endweights->getM(); i++)
		{
		   for(int j = 0; j < endweights->getN(); j++)
		   {
		         if ( !(file >> str))
		         {
		             std::cerr << "error while reading output weights file";
		             break;
		         }

		         if (str ==" ") break;

		         temp->val(i,j) = atof(str);
		   }
		   if ( !file ) break;
		}

		//this->*noise = *temp;

		for(int i = 0; i < endweights->getM(); i++)
				{
				   for(int j = 0; j < endweights->getN(); j++)

		             	this->endweights->val(i,j) = (temp->val(i,j));
				}

		delete temp;


}


void ESNetwork::resetInput()// sets the input neurons to zero
{
	std::cout<<"Resetting inputs:" << "\n";
	for (int i = 0; i < inputNeurons; i++)
	{
		inputs->val(i,0) = 0;
	}
	std::cout<<" \n done" << "\n";
}
void ESNetwork::scrambleInput() //randomizes input, unused
{
	for(int i = 0; i< inputNeurons; i++)
	{
		if (rand()%100 > 50)
		{
			inputs->val(i,0) = 1;
		}
		else
		{
			inputs->val(i,0) = 0;
		}
	}
}

//functions to print matrices to terminal
void ESNetwork::printMatrix(matrix::Matrix *printedMatrix)
{
	unsigned int i = 0;
	unsigned int j = 0;
	std::cout <<  "matrix 1 : \n";
	for(i = 0; i < printedMatrix->getM(); j++)
	{
		if (j == printedMatrix->getN())
		{
			j = 0;
			i++;
			std::cout << "\n";
			if (i == printedMatrix->getM()) {break;}
		}
		std::cout <<  printedMatrix->val(i,j) << " ";
	}
}
void ESNetwork::printOutputToTerminal()
{
	std::cout <<  "Output : ";
	for(unsigned int i = 0; i < outputs->getM();i++)
	{
		std::cout <<  outputs->val(i,0) << " ";
	}
	std::cout << "\n ";
}
void ESNetwork::printNeuronsToTerminal()
{
	std::cout <<  "Neurons : \n";
	for(unsigned  int i = 0; i < intermediates->getM();i++)
	{
		std::cout <<  intermediates->val(i,0) << " ";
	}
	std::cout << "\n ";
}


//restricts the range of neuron activation using sigmoid
void ESNetwork::cullInnerVector(float treshold /*unused*/)
{
	for(unsigned int i = 0; i < intermediates->getM(); i++)
	{

		if (nonlinearity == 0 )
			intermediates->val(i,0) = intermediates->val(i,0);

		else if (nonlinearity == 1 )
		{intermediates->val(i,0) = ESNetwork::sigmoid(intermediates->val(i,0));}

		else if (nonlinearity == 2)
		intermediates->val(i,0) = ESNetwork::tanh(intermediates->val(i,0),enable_IP);

		//Keep flag set to true for intrinsic plasticity, otherwise False

		  if (intermediates->val(i,0)*intermediates->val(i,0)<=0.0001) intermediates->val(i,0) = 0;
	}
}

void ESNetwork::cullOutput(float treshold /*unused*/)
{
	for( unsigned int i = 0; i < outputs->getM(); i++)
	{

	 if (outnonlinearity == 0) outputs->val(i,0) = outputs->val(i,0);

	else if (outnonlinearity == 1 )
			  outputs->val(i,0) = ESNetwork::sigmoid(outputs->val(i,0));
	else if (outnonlinearity == 2)
		     outputs->val(i,0) = ESNetwork::tanh(outputs->val(i,0), false);

			 // if (outputs->val(i,0)*outputs->val(i,0)<=0.0001) outputs->val(i,0) = 0;
	}
}
float ESNetwork::sigmoid(float x)
{
	return (1/(1 + exp(-x)));

}
float ESNetwork::deriSigmoid(float x)
{
	if (x < -3) {return 0;}
	if (x > 3) {return 0;}
	return (x*x-9)*(x*x-9)/(9*(x*x+3)*(x*x+3));
}

double ESNetwork::tanh(double x, bool flag)
{
	// Gaussian Intrinsic plasticity rule

	//static int a=0;
	//static int b=0; Flag is for enabling or disabling IP;

	if(flag == true)
	{
		double eta = 0.001; // learning rate for IP stochastic rule

		para_a = 1.0; para_b = 0.0;



		double y =2./(1.+exp(-2*(para_a*x+para_b)))-1.;

		if(enable_IP)
		{
			double del_b = updateGauss_gain(y, eta);

			para_b = para_b + del_b;

			para_a = para_a + ((eta/para_a)+ del_b*x);

			//	std::cout<<"para_b = "<<para_b;
		}

		//store_para_a[a++] = para_a;
		//store_para_b[b++]= para_b;

		return y;

	}

	else
		return 2./(1.+exp(-2*x))-1.;




}

double ESNetwork::updateGauss_gain(double y, double eta)
{
	double del_b= 0.0;
	double mu = 0.2; // mean of Gaussian distribution (mean firing rate)
	double sigma = 0.01;  // standard deviation

	del_b = -eta*(-(mu/(sigma*sigma)) + (y/(sigma*sigma))*(2*sigma*sigma +1 - (y*y)+ (mu*y)));

	return del_b;
}



void ESNetwork::normalizeInnerWeights(float density)
{
	matrix::Matrix * Eigens = new matrix::Matrix;
	*Eigens =  eigenValuesRealSym(*innerweights);

	// Workaround for Eigen value decomposition NULL matrix issue from LPZrobot (stupid but works ;))
	while(Eigens->val(0,0)== 0)
		*Eigens =  eigenValuesRealSym(*innerweights);

	printMatrix(Eigens);
	innerweights->mult(*innerweights, (1/Eigens->val(0,0))*density);
	*Eigens =  eigenValuesRealSym(*innerweights);
	std::cout<<std::endl;
	printMatrix(Eigens);
	delete Eigens;
}

void ESNetwork::normalizeInputWeights(float density)
{
	matrix::Matrix * Eigens = new matrix::Matrix;
		*Eigens =  eigenValuesRealSym(*startweights);

		// Workaround for Eigen value decomposition NULL matrix issue from LPZrobot (stupid but works ;))
		while(Eigens->val(0,0)== 0)
			*Eigens =  eigenValuesRealSym(*startweights);

		printMatrix(Eigens);
		startweights->mult(*startweights, (1/Eigens->val(0,0))*density);
		*Eigens =  eigenValuesRealSym(*startweights);
		std::cout<<std::endl;
		printMatrix(Eigens);
		delete Eigens;
}

float * ESNetwork::readOutputs()
{
	float * outputvalues = new float[outputNeurons];
	for(int i = 0;i < outputNeurons;)
	{
		outputvalues[i] = outputs->val(i,0);
	}
	return outputvalues;
}

double ESNetwork::uniform(double a, double b)
{
	return rand()/(RAND_MAX +1.0)*(b-a)+a;
}



/*
    ESNetwork::~ESNetwork()
    {
    	 delete inputs;// = new matrix::Matrix(inputNeurons,1);
    	 delete      outputs;// = new matrix::Matrix(outputNeurons,1);
    	 delete      intermediates;// = new matrix::Matrix(networkNeurons,1);


    	       //now the weights:
    	 delete     startweights;// = new matrix::Matrix(networkNeurons, inputNeurons);

    	        //innerweights is bigger if the output feeds back
    	        //int temp =  networkNeurons + (feedbackConnections ?  outputNeurons : 0);
    	        delete     innerweights; //= new matrix::Matrix(networkNeurons, temp);

    	        //endweigts is bigger, if the output feeds back
    	        //temp = (throughputConnections?  inputNeurons : 0)+ networkNeurons;
    	        delete      endweights; //  = new matrix::Matrix(outputNeurons, temp);


    }
 */
