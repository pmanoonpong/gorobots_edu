/************************************************************************
* \brief: genetic programming, solving the problem of prime factori-    *
*         zation. This is a minimum environment to allow genetic        *
*         programming based on machine language including crossover.    *
*																		                                    *
* (c) copyright by Joern Fischer											                    *
*                                                                       *
*  This source may be distributed freely without any warranty.          *
*  If it is used to produce data for publication of any kind            *
*  please cite any of my genetic programming papers related to this.    *
*																		                                    *
* @autor: Prof.Dr.Joern Fischer											                    *
* @email: j.fischer@hs-mannheim.de										                  *
*                                                                       *
* @file : main.cpp                                                      *
*************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "TinyVM.h"
#include "TinyGP.h"

#define MAX_PRIMES 168
#define MAX_GENERATIONS 100
#define MAX_POPULATION 100
#define MUTATION_RATE 0.1
#define CROSSOVER_RATE 0.5

int main(int argc, char** argv)
{
	double bestFitness;
	FILE* outFile;
	geneticProg myGP(MUTATION_RATE, CROSSOVER_RATE, MAX_POPULATION, MAX_PRIMES);

	outFile = fopen("fitness.csv","wb");

	fprintf(outFile,"Generation;bestFitness\n");
	for (int generation = 1; generation<=MAX_GENERATIONS; generation++){
		bestFitness = myGP.evolveGenerationAndGetBestFitness();
		printf("Generation %d fitness of best individuum = %f\n", generation, bestFitness);
		fprintf(outFile,"%d;%f\n", generation, bestFitness);
	}
	fclose(outFile);
	//myGP.simulateIndividuum(); // simulates best individuum
}
