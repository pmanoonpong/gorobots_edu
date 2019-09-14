//============================================================================
// Name        : Reconstructer.cpp
// Author      : Deniel Horvatic Hochschule Mannheim Germany
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include "time.h"
using namespace std;

#define MAX_TIME 500
#define MAX_NEURONS 10

static double weightMatrix[MAX_NEURONS + 1][MAX_NEURONS + 1];
static double calculatedWeightSolution[MAX_NEURONS];

static clock_t start, finish;

// counters for loops
static int n0, n1, t, n00, n11;	//some helper variables for loops

// initialize 4 control parameter
static double a = 0.02;	//affect recovery speed of u //higher value = fast --> and opposite
static double b = 0.2;		//affect u //how strong u and v are coupled together
static double c = -55;//-65;	//value of membrane recovery variable u after action potential occured
static double d = 4;			//affect afterspike reset difference of u

//initialize
static double I[MAX_NEURONS][MAX_TIME];	//Input current which is given if a connected neuron releases an action potential one time step before
static double v[MAX_NEURONS][MAX_TIME];			//membrane potential
static double u[MAX_NEURONS][MAX_TIME];			//membrane recovery variable
static double weight[MAX_NEURONS][MAX_NEURONS];	//real weight weightMatrix

// some test Variables
static double onlyInput[MAX_NEURONS][MAX_TIME];	//random input current between 0 - 29
static double Vfactor[MAX_NEURONS][MAX_TIME];		//Eugene Itzikevich's --> v'
static double Spike[MAX_NEURONS][MAX_TIME];	//help array for recognizing if a neuron spikes // values [0;1]

void gaussElimination(int lastColumn) {
	int n0, n1, n2, strongestNeuron;
	double equationLineToSwap, t;
	//FILE* gauss;
	//gauss = fopen("test.txt","wb");

	//pivot elimination
	for (n0 = 0; n0 < lastColumn; n0++) {
		// get highest absolute value from weightmatrix because of i.e. 0.00001
		strongestNeuron = n0;

		for (n1 = n0 + 1; n1 < lastColumn; n1++) {
			// choose highest for error reducing because of i.e. 0.00001
			if (fabs(weightMatrix[n1][n0])
					> fabs(weightMatrix[strongestNeuron][n0]))//fabs = method for calculating absolute value
				strongestNeuron = n1;
		}

		// swap with line of highest value (strongest connection)
		for (n2 = n0; n2 <= lastColumn; n2++) {
			equationLineToSwap = weightMatrix[n0][n2];
			weightMatrix[n0][n2] = weightMatrix[strongestNeuron][n2];
			weightMatrix[strongestNeuron][n2] = equationLineToSwap;
		}

		// eliminate each field below pivot diagonal
		for (n1 = n0 + 1; n1 < lastColumn; n1++) {//row		start at 1											//n1 = next (dimension + 1)
			for (n2 = lastColumn; n2 >= n0; n2--) {	//colums start at 103												//k =  backwards (10 - 0)
				// if weightMatrix[n,n] not eliminated --> weightMatrix[0,0,1,1,2,2]... != 0
				// do weightMatrix[n,m] = weightMatrix[n,m] - weightMatrix[d,n] * weightMatrix[n,d] / weightMatrix[d,d]
				if (weightMatrix[n0][n0] != 0) {
					weightMatrix[n1][n2] -= weightMatrix[n0][n2]
							* weightMatrix[n1][n0] / weightMatrix[n0][n0];
				}
				//do weightMatrix[n,m] = weightMatrix[n,m] - weightMatrix[d,m] * weightMatrix[n,d] / 0
				else { // == 0 --> why division trough zero???
					weightMatrix[n1][n2] -= weightMatrix[n0][n2]
							* weightMatrix[n1][n0] / 0.00000000000000000001;
					//printf("Division by Zero\n");
				}
			}
		}

		//for (n1=0;n1<=lastColumn;n1++){//row		start at 1											//n1 = next (dimension + 1)
		// for (n2=0;n2<lastColumn;n2++){				//colums start at 103												//k =  backwards (10 - 0)

		//	 fprintf(gauss,"%f ",matrix[neuron][k]);
		// }
		// fprintf(gauss,"\n");
		//}
		//fprintf(gauss,"----------------------------\n");

	}
	//fclose(gauss);
	//calculate weight solution with help of substitution
	for (n1 = 0; n1 < lastColumn; n1++)
		calculatedWeightSolution[n1] = 0;

	for (n1 = lastColumn - 1; n1 >= 0; n1--) {	//loop backwards
		// substitute
		t = 0.0;
		// t = t + matrix[n,m] * wSolution[m]
		for (n2 = n1 + 1; n2 < lastColumn; n2++) {
			t += weightMatrix[n1][n2] * calculatedWeightSolution[n2];
		}
		// if pivot element != 0 --> weightMatrix[0,0;1,1;2,2;...] != 0
		// do wSlution[n] = (weightMatrix[n,m] - t) / weightMatrix [n,n]
		if (weightMatrix[n1][n1] != 0) {
			calculatedWeightSolution[n1] = (weightMatrix[n1][lastColumn] - t)
					/ weightMatrix[n1][n1];
			// wSlution[n] = (matrix[n,m] - t) / 0 --> Why trough zero???
		} else { // == 0
			calculatedWeightSolution[n1] = (weightMatrix[n1][lastColumn] - t)
					/ 0.000000000000000000001;
		}
	}

}

void spikingNeuronSimulation() {
	// opens window with size 500,410
	// Machine.Start(1000,800);

	//	cout << "\n\n\n\n U\n\n\n\n\n\n\n\n V[mV]\n\n\n\n\n\n\n\n\n\n\n I[mV]";
	//	Machine.Line(0,170,1000,170,RGB(100,100,100));
	//Machine.Line(0,200,1000,200,RGB(200,200,200));
	//Machine.Line(0,101,1000,101,RGB(200,0,0));
	//Machine.Line(0,400,1000,400,RGB(0,50,0));

	// set initial membrane potential / recovery variable and random weights //timestep = 0
	for (n0 = 0; n0 < MAX_NEURONS; n0++) {// all neurons start at the same point

		v[n0][0] = c;	//set each neuron membrane potential at beginning to c
		u[n0][0] = b * c;	//set each neuron membrane recovery variable to b*c

		// set random weights
		for (n1 = 0; n1 < MAX_NEURONS; n1++) {

			// excitatory random weights
			weight[n1][n0] = (rand() % 10000) / 1000.0;	// random values between 0 and 9.99999999999

			if (n1 >= 7)
				weight[n1][n0] *= -1.0;		// inhibitory

			if (n1 == n0)
				weight[n1][n0] = 0; 		// no self connection!

		}
	}

	//go on with simulation at 1 - 999
	for (t = 1; t < MAX_TIME; t++) {			//delta is 1

		for (n0 = 0; n0 < MAX_NEURONS; n0++) {

			// set I to random value for each neuron and timestep
			onlyInput[n0][t] = rand() % 30;
			I[n0][t] = onlyInput[n0][t];// set I[neuron][timestep] = to random value between 0 and 29

			// check for each connected neuron if neuron spikes and set I
			for (n1 = 0; n1 < MAX_NEURONS; n1++) {

				Spike[n1][t] = 0;	// set each spike at the beginning to zero

				if (v[n1][t - 1] >= 30) {	// check if spiked before
					Spike[n1][t] = 1.0;	//set spike to 1.0
				}
				// set input current to (0 or 1) * weight
				I[n0][t] += Spike[n1][t] * weight[n0][n1];
			}

			// v' = v[t-1] + 0.5(0.04v[t-1]² + 5v[t-1] + 140 - u[t-1])
			Vfactor[n0][t] =
					v[n0][t - 1]
							+ 0.5
									* (0.04 * v[n0][t - 1] * v[n0][t - 1]
											+ 5.0 * v[n0][t - 1] + 140.0
											- u[n0][t - 1]);//eugene itzikevich equation 4.3

							// calculate membrane potential v
							// v[t] =  v[t-1] + 0.5 * ( 0.04*v[t-1]² + 5*v[t-1] + 140 - u[t-1] )
			v[n0][t] = Vfactor[n0][t] + 0.5 * I[n0][t]; // which equation???

			// trim reached thresholds to 30
			if (v[n0][t] >= 30) {
				v[n0][t] = 30;
			}
			// calculate membrane recovery variable u
			// ut = ut-1 + 0.5 * a * ( b * vt-1 - ut-1)
			u[n0][t] = u[n0][t - 1]
					+ 0.5 * a * (b * v[n0][t - 1] - u[n0][t - 1]); //eugene itzikevich eqution 4.4

					// reset condition if membrane potential threshold one timestep before is reached
			if (v[n0][t - 1] == 30) {
				v[n0][t] = c;
				u[n0][t] = u[n0][t - 1] + d;
			}
		}

		//draw lines
		//for each neuron draw ???
		//for (neuron=1;neuron<10;neuron++){
		//	Machine.Line(timeStep,200-v[neuron][timeStep],timeStep,200-v[neuron][timeStep-1],RGB(0,0,255));
		//}
		//Machine.Line(timeStep,200-v[0][timeStep],timeStep,200-v[0][timeStep-1],RGB(255,255,255));			//draw v
		//Machine.Line(timeStep,100-u[0][timeStep],timeStep,100-u[0][timeStep-1],RGB(255,0,0));				//draw u
		//Machine.Line(timeStep,400-I[0][timeStep],timeStep,400-I[0][timeStep-1],RGB(0,255,0));				//I
	}
}

void reconstruction() {
	// create files
	FILE *weightMatrixFile, *errorFile;
	weightMatrixFile = fopen("weightMatrix.txt", "wb");

	errorFile = fopen("error.txt", "wb");
	bool hasSolutionFound = false;
	double totalError = 0;

	// variables for reconstruction
	double Iguess[MAX_NEURONS][MAX_TIME];		//I after reconstruction
	double SpikeGuess[MAX_NEURONS][MAX_TIME];	//check if spiked
	double Wguess[MAX_NEURONS][MAX_NEURONS];//weightmatrix after reconstruction
	double Uguess[MAX_NEURONS][MAX_TIME];//membrane potential u after reconstruction
	double VfactorGuess[MAX_NEURONS][MAX_TIME];	//membrane potential without I

	//helper variable
	double vOneStepBefore = 0;

	// ---------------------- here starts the reconstruction --------------------------
	for (double aa = 0.01; aa < 0.1 && !hasSolutionFound; aa += 0.002) {

		for (double bb = 0; bb < 0.3 && !hasSolutionFound; bb += 0.01) {

			// Fill IsSpiked array --> v from simulation is given
			for (n0 = 0; n0 < MAX_NEURONS; n0++) {

				for (t = 1; t < MAX_TIME; t++) {
					//possible to calculate c here !!!
					SpikeGuess[n0][t] = 0;//set spikeGuess value to initial zero

					//is guessed neuron spiking
					if (v[n0][t - 1] >= 30)
						SpikeGuess[n0][t] = 1.0;// Value is multiplied with the weight ti get I

					// check if guessed spike is right
					if (SpikeGuess[n0][t] != Spike[n0][t])
						cout << "spike incorrect";

				}
			}

			// Reconstruction of u and vfactor --> v and original u0 from simulation is given
			for (n0 = 0; n0 < MAX_NEURONS; n0++) {

				for (t = 0; t < MAX_TIME; t++) {

					//set guessed[0] parameteres (Uguess,a,b) to original values if initial timeStep
					// Uguess[o] = b * c
					if (t == 0) {
						Uguess[n0][0] = u[n0][0];// parameter to guess set to original
						a = aa;							// parameter to guess
						b = bb;							// parameter to guess
						continue;
					}
					//calculate Uguess
					// Uguess[t] = Uguess[t-1] + 0.5 * a * (b * v[t-1] - Uguess[t-1])
					Uguess[n0][t] = Uguess[n0][t - 1]
							+ 0.5 * a * (b * v[n0][t - 1] - Uguess[n0][t - 1]);

					//add d to uguess if membrane potential threshold is reached (neuron spiked)
					if (v[n0][t - 1] == 30)
						Uguess[n0][t] = Uguess[n0][t - 1] + d;

					//calculate Vfactorguess, because we don't know the weight
					vOneStepBefore = v[n0][t - 1];
					// v'[t]-I = v[t-1] + 0.5 * v'[t-1]
					// v'[t]-I = v[t-1] + 0.5 * (0.04v[t-1]² + 5v[t-1] + 140 - u[t-1] )
					VfactorGuess[n0][t] = vOneStepBefore
							+ 0.5
									* (0.04 * vOneStepBefore * vOneStepBefore
											+ 5.0 * vOneStepBefore + 140.0
											- Uguess[n0][t - 1]);
				}
			}

			//fill equation system --> given: v and vfactorguess and InputCurrent --> equation 4.8
			for (int n0 = 0; n0 < MAX_NEURONS; n0++) {

				// for resetting weightMatrix
				for (n1 = 0; n1 < MAX_NEURONS; n1++) {
					for (n11 = 0; n11 <= MAX_NEURONS; n11++) {
						weightMatrix[n1][n11] = 0;
					}
				}

				// for each timestep where neuron at t and t-1 not spiked
				for (int t = 1; t < MAX_TIME; t++) {

					//calculate only if neuron not spiked at t-1 and t // maybe because if spiked we'll use reset condition for v and u
					if (v[n0][t - 1] >= 30 || v[n0][t] >= 30)
						continue;

					//w[n0,10]  --> 2* (v[n,t] - v'Guess[n,t])-I[n,t] --> 2.0 because equation is multiplied by 2 to get rid of factor 0.5
					double wSolution = 2.0 * (v[n0][t] - VfactorGuess[n0][t])
							- onlyInput[n0][t];

					//--------------- Solve ----------------
					//weightMatrix[n,10] = weightMatrix[n,10] + 2*SpikeGuess*(2* (v[n,t] -v'Guess[n,t])-I[n,t])
					//for each last neuron in weightMatrix (weightMatrix[n,10])
					for (n00 = 0; n00 < MAX_NEURONS; n00++) {
						weightMatrix[n00][MAX_NEURONS] += 2.0
								* SpikeGuess[n00][t] * wSolution;
					}
					//fill weightMatrix A
					//weightMatrix[n,m] = weightMatrix[n,m] + 2 * SpikeGuess[n,t] * SpikeGuess[m,t] for each neuron in the weightMatrix
					for (n00 = 0; n00 < MAX_NEURONS; n00++) {
						for (n11 = 0; n11 < MAX_NEURONS; n11++) {
							weightMatrix[n00][n11] += 2.0 * SpikeGuess[n00][t]
									* SpikeGuess[n11][t];
						}
					}

				}

				gaussElimination(MAX_NEURONS); //solve equation system

				fprintf(weightMatrixFile, "param a = %f  param b = %f\n", aa,
					bb);
				//write real and calculated weight in txt file
				for (n1 = 0; n1 < MAX_NEURONS; n1++) {
					fprintf(weightMatrixFile,
						"realWeight = %f  calcWeight = %f\n",
						weight[n0][n1], calculatedWeightSolution[n1]);

					Wguess[n0][n1] = calculatedWeightSolution[n1];
				}

				fprintf(weightMatrixFile, "\n");

			}
			// ---------------------- start of error calculation ---------------------------------------
			totalError = 0;

			for (t = 0; t < MAX_TIME; t++) {

				for (int n0 = 0; n0 < MAX_NEURONS; n0++) {

					if (t == 0)
						continue;

					Iguess[n0][t] = onlyInput[n0][t]; // what if we delete this?????????????????????????????????????????????? asked by andreas

					// Iguess[n,t] = Iguess[n,t] + SpikeGuess[m,t] * Wguess[n,m]
					for (n00 = 0; n00 < MAX_NEURONS; n00++)
						Iguess[n0][t] += SpikeGuess[n00][t] * Wguess[n0][n00];

					//double vGuess = VfactorGuess[n0, t] + 0.5I;
					//double vGuess = VfactorGuess[n0, t] + 0.5( onlyInput[n0]+SUM( W[n0,n009) ) );
					double vGuess = VfactorGuess[n0][t] + 0.5 * Iguess[n0][t];

					// does it spike one timestep before?
					if (v[n0][t - 1] == 30)
						vGuess = c;

					if (vGuess >= 30)
						vGuess = 30;

					double vv = v[n0][t];
					// error = error + (v'guess-v)²
					totalError += (vGuess - vv) * (vGuess - vv);
				}

			}

			//cout << "calculation for a: " << aa << " and b: " << bb << " and error: " << totalError << "\n";
			fprintf(errorFile, "%f,%f,%f \n", aa, bb, totalError);

		} // end aa
	} // end bb

	fclose(weightMatrixFile);
	fclose(errorFile);
}

void printInputAndOutputFile() {
	FILE *inputFile, *outputFile;
	outputFile = fopen("outputFile.txt", "wb");
	inputFile = fopen("inputFile.txt", "wb");
	double input = 0, output = 0;
	for (int i = 0; i < MAX_TIME; i++) {
		input = onlyInput[0][i];
		output = I[0][i];
		if (output >= 30)
			output = 30;
		fprintf(inputFile, "%d\t%f\n", i, input);
		fprintf(outputFile, "%d\t%f\n", i, output);
	}
	fclose(inputFile);
	fclose(outputFile);
}

int main() {
	cout << "!!!Start Simulation!!!" << endl;
	spikingNeuronSimulation();
	cout << "!!!End   Simulation!!!" << endl;

	cout << "!!!Start Reconstruction!!!" << endl;
	start = clock(); 		//for measuring duration of recontruction
	reconstruction();
	finish = clock();
	cout << "Time for reconstruction (seconds): "
			<< ((double) (finish - start)) / CLOCKS_PER_SEC << "\n";
	cout << "!!!End   Reconstruction!!!" << endl;

	printInputAndOutputFile();
	return 0;
}
