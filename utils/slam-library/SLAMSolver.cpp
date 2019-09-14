#include "SLAMSolver.h"

namespace SLAM{

SLAMSolver::SLAMSolver(std::vector<Particle> initBelief, MeasurementModel* measureModel, MotionModel* motionModel, MapModel* mapModel){
	this->init(initBelief, measureModel, motionModel, mapModel);
	this->useKLD = false;
	this->kldParam = NULL;
}

SLAMSolver::SLAMSolver(std::vector<Particle> initBelief, MeasurementModel* measureModel, MotionModel* motionModel, MapModel* mapModel,
		KLDParameters* kldParam){
	this->init(initBelief, measureModel, motionModel, mapModel);
	this->kldParam = kldParam;
	this->useKLD = true;
	this->weightsBelief.resize(initBelief.size());
	this->sumWeights = 0;
	for(unsigned int i = 0; i < weightsBelief.size(); i++){
		this->weightsBelief[i] = initBelief[i].weight;
		this->sumWeights += this->weightsBelief[i];
	}
}

void SLAMSolver::init(std::vector<Particle> initBelief, MeasurementModel* measureModel, MotionModel* motionModel, MapModel* mapModel){
	this->numberParticles = initBelief.size();
	this->currentBelief = initBelief;		
	this->mapModel = mapModel;
	this->measureModel = measureModel;
	this->motionModel = motionModel;
	this->tools = new Tools();
}

bool SLAMSolver::isUsingKLD(){
	return this->useKLD;
}
void SLAMSolver::disableKLD(){
	this->useKLD = false;
}

void SLAMSolver::enableKLD(KLDParameters* kldParam){
	this->kldParam = kldParam;
	this->useKLD = true;
}

void SLAMSolver::fastSlamStep(const std::vector<double>& control, const std::vector<double>& measurement){
	//First decide, if to use KLD sampling or not
	if(this->useKLD)
		//Pass an arbitrary map to particle filter, as it will be ignored anyway, when first parameter is true
		this->particleFilterKLD(true, control, measurement, this->currentBelief[0].map);
	else
		this->particleFilterNaive(true, control, measurement, this->currentBelief[0].map);
}

void SLAMSolver::MCL_step(const std::vector<double>& control, const std::vector<double>& measurement, const Grid<double>& m){
	//First decide if to use KLD sampling or not
	if(this->useKLD)
		this->particleFilterKLD(false, control, measurement, m);
	else
		this->particleFilterNaive(false, control, measurement, m);
}

void SLAMSolver::particleFilterNaive(bool doSLAM, const std::vector<double>& control, const std::vector<double>& measurement, 
		const Grid<double>& m){
	std::vector<double> weights(this->numberParticles); //Weights according to measurement model are stored here
	std::vector<double> state;
	std::vector<Particle> tempBelief(this->numberParticles);
	double sumWeight = 0; //Sum up weights to be able to draw particles more efficiently
	for(int i = 0; i < this->numberParticles; i++){
		//Sample according to motion model
		state = this->motionModel->sample(control, this->currentBelief[i].state);
		//Compute weights based on measurement
		if(doSLAM)
			//For SLAMSolver problem use map of individual particle
			weights[i] = this->measureModel->computeWeight(measurement, state, this->currentBelief[i].map);
		else
			//For MCL problem use given map
			weights[i] =this->measureModel->computeWeight(measurement, state, m);
		//Sum up weights
		sumWeight += weights[i];
		//Update map when doing SLAMSolver
		if(doSLAM)
			this->mapModel->updateMap(measurement, state, this->currentBelief[i].map);
		//Add particle to temp list
		tempBelief[i] = Particle(state, weights[i], this->currentBelief[i].map);
	}

	int index;
	//Draw particles according to their weights and update current belief of robot
	for(int i = 0; i < this->numberParticles; i++){
		index = this->tools->drawWeights(weights,sumWeight);
		currentBelief[i] = tempBelief[index];
	}
}
void SLAMSolver::particleFilterKLD(bool doSLAM, const std::vector<double>& control, const std::vector<double>& measurement, const Grid<double>& m){
	//Vector containing all particles of the belief generated in this method
	std::vector<Particle> newBelief;
	//Log weights of new belief for easier sampling with drawWeights
	std::vector<double> newWeights;
	//Log sum of weights for easier sampling with drawWeights
	double newSumWeights = 0;
	//Counts occupied grid cells
	int k = 0;
	//Temp variables
	double M_Chi = 0;
	bool value;
	bool indexCorrect;
	int particleIndex, binIndex;
	std::vector<double> state;
	double weight,temp1, temp2;
	//Reset number particles
	this->numberParticles = 0;
	//Reset state space representation, such that all cells are set to free
	this->kldParam->gridStateSpace.setAllCells(false);
	//Start adding particles
	do{
		//Draw new particle according to their weights
		particleIndex = this->tools->drawWeights(this->weightsBelief, this->sumWeights);
		//Sample according to motion model
		state = this->motionModel->sample(control, this->currentBelief[particleIndex].state);
		//Compute weights based on measurement
		if(doSLAM)
			//For SLAMSolver problem use map of individual particle
			weight = this->measureModel->computeWeight(measurement, state, this->currentBelief[particleIndex].map);
		else
			//For SLAMSolver problem use given map
			weight = this->measureModel->computeWeight(measurement, state, m);
		//Update map when doing SLAMSolver
		if(doSLAM)
			this->mapModel->updateMap(measurement, state, this->currentBelief[particleIndex].map);
		//Log new weight to reduce computation time
		newSumWeights += weight;
		newWeights.push_back(weight);
		//Store new particles
		newBelief.push_back(Particle(state,weight,this->currentBelief[particleIndex].map));
		//Find cell of current particle in state space representation
		binIndex = this->kldParam->gridStateSpace.getCellIndex(state);
		//Get value of cell
		indexCorrect = this->kldParam->gridStateSpace.getCellValue(binIndex, value);
		//Check if cell is occupied and if state was in cell
		if(indexCorrect && !value){
			//If not, set cell to occupied and update k
			this->kldParam->gridStateSpace.setCellValue(binIndex, true);
			k++;
			//Estimate if enough particles are present by computing the "volume" that is covered by the particles
			if(k > 1){
				temp1 = 2./(9.*(k-1));
				temp2 = 1 - temp1 + sqrt(temp1) * quantile(this->kldParam->normalDistribution, 1 - this->kldParam->sigma);
				M_Chi = (k - 1) / (2*this->kldParam->epsilon) * temp2 * temp2 * temp2;
			}
		}
		this->numberParticles++;
	} 
	while(this->numberParticles < M_Chi || this->numberParticles < this->kldParam->minNumberParticles);
	//Copy results to member variables
	this->weightsBelief = newWeights;
	this->sumWeights = newSumWeights;
	this->currentBelief = newBelief;
}


void SLAMSolver::writeBelief(std::string fileName){
	std::ofstream file;
	file.open(fileName.c_str());
	Particle p;
	if(file.is_open()){
		for(unsigned int i = 0; i < this->currentBelief.size(); i++){
			p = this->currentBelief[i];
			file << i << "\t";
			for(unsigned j = 0; j < p.state.size(); j++)
				file <<p.state[j]<<"\t";
			file << p.weight<< std::endl;
		}
	}
	file.close();
}

std::vector<Particle> SLAMSolver::getBelief(){
	return this->currentBelief;
}

}
