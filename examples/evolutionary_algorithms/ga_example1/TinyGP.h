/************************************************************************
* \brief: genetic programming, solving the problem of prime factori-    *
*         zation. This is a minimum environment to allow genetic        *
*         programming based on machine language including crossover.    *
*																		                                    *
* (c) copyright by Joern Fischer											                  *
*                                                                       *
*  This source may be distributed freely without any warranty.          *
*  If it is used to produce data for publication of any kind            *
*  please cite any of my genetic programming papers related to this.    *
*																		                                    *
* @autor: Prof.Dr.Joern Fischer											                    *
* @email: j.fischer@hs-mannheim.de										                  *
*                                                                       *
* @file : TinyGP.h                                                      *
*************************************************************************/

#ifndef TINY_GP
#define TINY_GP

class geneticProg
{
public:
			
	TinyVM *myVM;
	int populationSize;
	double mutationRate;
	double crossoverRate;
	int numberOfGenerations;
	int *primeNumber; 
	int numberOfPrimes;
	#define PROTECT_BEST 5

	/**********************************************************************
	* \brief: constructor: initializes variables and generates random 
	*         programs and computes a number of primes
	**********************************************************************/
	geneticProg(double mutationRate = 0.1, double crossoverRate = 0.5, int populationSize = 100, int numberOfPrimes = 168)
	{
		this->populationSize = populationSize;
		this->numberOfPrimes = numberOfPrimes;
		this->mutationRate = mutationRate;
		this->crossoverRate = crossoverRate;

		primeNumber = new int[numberOfPrimes];
		myVM = new TinyVM[populationSize];

		// Ininialize Individuums
		for (int individuum=0; individuum<populationSize; individuum++){
			for (int t=0;t<PROG_LENGTH;t++){ 
				myVM[individuum].memory[t]=rand(); // random programs
			}
			for (int t=SUBROUTINE_SIZE; t<PROG_LENGTH; t+=SUBROUTINE_SIZE){ 
				myVM[individuum].memory[t-1]=RET; // end of subroutines
			}
		}
		// compute prime numbers
		int counter;
		counter = 0;
		for (int t=2;t<1000000;t++){
			int SQR = sqrt((double)t);
			bool prime = true;
			for (int r=2; r<=SQR; r++){
				if ((t % r) == 0){
					prime = false;
					r = SQR;
				}
			}
			if (prime){
				primeNumber[counter++] = t;
				printf("%d,",t);
			}
			if (counter >= (numberOfPrimes-1)){
				printf("\n");
				break;
			}
		}
		
	}
	/**********************************************************************
	* \brief: destructor prevents memory leaks
	**********************************************************************/
	~geneticProg()
	{
		delete[] primeNumber;
		delete[] myVM;
	}
	/**********************************************************************
	* \brief: evaluates the fitness of all individuals
	**********************************************************************/
	void evaluateFitness(int individuum)
	{
		for (int zz=0;zz<100;zz++){
			myVM[individuum].primeA = primeNumber[rand()%numberOfPrimes];
			myVM[individuum].primeB = primeNumber[rand()%numberOfPrimes];
	
			int cycles = myVM[individuum].simulate();
		}
	}
	/**********************************************************************
	* \brief: sorts the individuals by fitness
	**********************************************************************/
	void selectBestIndividuals()
	{
		// sort individuals for fitness
		for (int t=0;t<populationSize;t++){
			for (int i=t+1;i<populationSize;i++){
				if (myVM[t].fitness < myVM[i].fitness){
					double help;
					help = myVM[t].fitness;
					myVM[t].fitness = myVM[i].fitness;
					myVM[i].fitness = help;
					for (int j=0; j<PROG_LENGTH; j++){
						unsigned short mem;
						mem = myVM[t].memory[j];
						myVM[t].memory[j] = myVM[i].memory[j];
						myVM[i].memory[j] = mem;
					} // endfor
				}// endif
			}// endfor i
		}// endfor t
	}
	/**********************************************************************
	* \brief: half of the individuals of the next generation are produced
	*         by crossover...
	**********************************************************************/
	void crossover()
	{
		int crossoverPoint, loverIndividuum;
		// --- better half generates children into the worse half
		for (int t=0;t<populationSize*crossoverRate;t++){
			//myVM[t+populationSize/2].fitness = myVM[t].fitness;
		
			crossoverPoint  = (rand()%PROG_LENGTH * SUBROUTINE_SIZE) % PROG_LENGTH;
			loverIndividuum = rand() % populationSize; // gets children with random 
		
			// half of selected are build via crossover

			for (int j=0;j<PROG_LENGTH;j++){
				if (j<crossoverPoint){
					myVM[(int)(t+populationSize*(1.0-crossoverRate))].memory[j] = myVM[t].memory[j]; // mother gene
				}
				else{
					myVM[(int)(t+populationSize*(1.0-crossoverRate))].memory[j] = myVM[loverIndividuum].memory[j]; // father gene
				}
			}
		}
	}
	/**********************************************************************
	* \brief: sets all fitness values to zero
	**********************************************************************/
	void resetFitness()
	{
		for (int t=0;t<populationSize;t++){
			myVM[t].fitness = 0;
		}
	}
	/**********************************************************************
	* \brief: mutates some of the individuals dependent on the mutation rate
	**********************************************************************/
	void mutate()
	{
		for (int t=0; t<populationSize * mutationRate; t++){ // 10% mutation
			myVM[rand()%(populationSize-PROTECT_BEST)+PROTECT_BEST].memory[rand()%PROG_LENGTH] = rand(); // Mutate, but save the best 5
		}
		// over all individuums: inshure that "return from subroutine" is not mutated
		for (int individuum=0; individuum<populationSize; individuum++){
			for (int t=SUBROUTINE_SIZE; t<PROG_LENGTH; t+=SUBROUTINE_SIZE){ 
				myVM[individuum].memory[t-1]=RET; // end of subroutines
			}
		}
	}
	/**********************************************************************
	* \brief: The steering of the evolution process is done here
	**********************************************************************/
	double evolveGenerationAndGetBestFitness()
	{
		int individuum=0;

		resetFitness();
		
		// over all individuums: 
		for (int individuum=0; individuum<populationSize; individuum++){
			evaluateFitness(individuum);
		}
	
		selectBestIndividuals();
		crossover();
		mutate();	

		return (double)myVM[0].fitness/100.0;		
	}
	/**********************************************************************
	* \brief: This function enables the user to view what the evolved 
	*         program does
	**********************************************************************/
	int simulateIndividuum(int individuum = 0, bool debug = true)
	{
		myVM[individuum%populationSize].debug = debug;
		int cycles = myVM[individuum%populationSize].simulate();
		myVM[individuum%populationSize].debug = false;
		return cycles;
	}

};
#endif //TINY_GP
