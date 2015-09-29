//Include necessary SLAM files
#include "grid.h"
#include "SLAMSolver.h"
#include "tools.h"
#include "rangeFinder.h"
#include "Models/occupancyGridBresenham.h"
#include "Models/odometryModel.h"
#include "Models/velocityModel.h"
#include "Models/likelihoodField.h"
//Standard library
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
//Boost
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/assign/list_of.hpp>

using namespace SLAM;
using namespace std;
using namespace boost;


//This example illustrates the use of the the SLAMSolver class for localization tasks. For a more detailed and general introduction
//to this class and the underlying algorithms, look at the manual in examples/SLAM/manual.pdf and the comments in utils/SLAM/SLAMSolver.h.
//The test data utilized in this example was obtained with a slightly modified version of the Nimm2 robot. The controller can be found
//in projects/SLAM/Basic.
//To visualize results, run 'python plotSLAM.py path stepNumber', which will show the computed and true path up to step stepNumber.
//Further information can be found in the manual.

//Read output from the LRF and store in vector. Furthermore velocity and position of the robot are read (obtained through
//simulation).
//tools: Instance of tool class. Only used for sampling from normal distribution here
//fileName: Name of the file containing the data
//variancePos: Variance of the noise added to the position values
//varianceVelo: Variance of the noise added to the forward velocity
//varianceGyro: Variance of the noise added to the gyroscope values
//states: Vector is overwritten with 'true' states of the robot. Each entry is of the form:
//			states[i][0]: Position x
//			states[i][1]: Position y
//			states[i][2]: Orientation angle (angle with respect to x-axis)
//states: Vector is overwritten with measurements of laser range finder. Each entry is one complete scan
//velocities: Vector is overwritten with 'true' velocities of the robot. Each entry is of the form:
//			velocities[i][0]: Forward velocity
//			velocities[i][1]: Rotational velocity around z-axis of robot
//Return: Vector, containing the indices of the measurements
vector<int> readData(Tools* tools, string fileName, 	double variancePos, double varianceVelo, double varianceGyro,
		vector<vector<double> >& states, vector<vector<double> >& measurements,
		vector<vector<double> >& velocities){

	states.clear();
	measurements.clear();
	velocities.clear();

	ifstream myfile(fileName.c_str());
	string line;
	vector<string> strs;
	vector<double> z;
	vector<int> indices;
	vector<double> s(3);
	vector<double> v(2);
	while(getline(myfile,line))
	{
		if(line[0]=='#')
			continue;
		split(strs,line, boost::is_any_of("\t"));
		indices.push_back(lexical_cast<int>(strs[0]));
		//Read states
		s[0] = lexical_cast<double>(strs[1]) + tools->sampleNormal(variancePos);
		s[1] = lexical_cast<double>(strs[2]) + tools->sampleNormal(variancePos);
		s[2] =  lexical_cast<double>(strs[3]);
		states.push_back(s);
		//REad velocities
		v[0] = lexical_cast<double>(strs[4]) + tools->sampleNormal(varianceVelo);
		//Entry with index 5 is velocity perpendicular to forward
		v[1] = lexical_cast<double>(strs[6]) + tools->sampleNormal(varianceGyro);
		velocities.push_back(v);
		//Read LRF
		for(int j = 9; j < 24;j++){
			try{
				z.push_back(lexical_cast<double>(strs[j]));
			}
			catch(bad_lexical_cast const&){
				cout << "Error casting value to double"<< endl;
				continue;
			}
		}
		measurements.push_back(z);
		z.clear();

	}
	myfile.close();
	return indices;
}


int main(int argc, char *argv[])
{
	Tools tools(1);

	//Filenames
	string fileNameSensor = "sensorData/sensorData.dat";
	string fileNameAcc = "sensorData/accData.dat";

	//Which motion model to use
	int useVelocity = false;

	//Model noise. If changed, noise parameters should be adjusted in parameters.cfg
	double variancePos = 0.1;
	double varianceVelo = 0.1;
	double varianceRot = 0.05;

	//Read data from files
	vector<vector<double> > states;
	vector<vector<double> > measurements;
	vector<vector<double> > angularVelocities;
	vector<int> indices = readData(&tools, fileNameSensor, variancePos, varianceVelo, varianceRot,
			states, measurements, angularVelocities);

	//Load Map
	Grid<double> m("circle.map");

	//All parameters needed for the individual models are read from files with libconfig++. For documentation on the meaning
	//of parameters, see parameters.cfg and the comments in the corresponding header files in utils/SLAM/Models
	const char* cfgFileName =  "parameters.cfg"; //Here all necessary parameters are stored in one file
	RangeFinder rf(cfgFileName);
	OdometryModel odometryModel(cfgFileName);
	VelocityModel velocityModel(cfgFileName);
	LikelihoodField lField(cfgFileName,&rf);
	OccupancyGridBresenham occMap(cfgFileName,&rf);
	KLDParameters kldParam(cfgFileName);

	double variance = 0.2; //Variance of initial states
	//Starting position of robot
	double startX = states[0][0];
	double startY = states[0][1];
	double orientation = states[0][2];
	int numberParticles = 1000; //Initial number of particles
	//Set initial belief
	double x,y;
	vector<Particle> particles(numberParticles);
	vector<double> s;
	for(int i = 0; i < numberParticles; i++){
		x = tools.sampleNormal(startX, variance);
		y = tools.sampleNormal(startY, variance);
		s = assign::list_of(x)(y)(orientation);
		//All particles have equal initial weights of 1/n
		particles[i] = Particle(s,1./numberParticles,m);
	}

	//Init SLAMSolver
	SLAMSolver mySLAM;
	//With velocity model
	if(useVelocity)
		mySLAM = SLAMSolver(particles,&lField, &velocityModel, &occMap, &kldParam);
	//With odometry model
	else
		mySLAM = SLAMSolver(particles,&lField, &odometryModel, &occMap, &kldParam);

	//Init logging of results and time
	ostringstream oss;
	clock_t t_begin = clock();
	ofstream out("data/particles.dat");
	out << "#Index\tPosX\tPosY\tTruePosX\tTruePosY\tYaw\tTrueYaw"<<endl;

	vector<double> control;
	for(int i = 0; i < (int)states.size();i++){
		cout << "Step "<<i<<" of "<<states.size()<<endl;
		if(i != 0){ //First step is skipped, because odometry model needs previous state
			//Set control value depending on model
			if(useVelocity){
				control = angularVelocities[i];
			}
			else{
				control = states[i];
				control.insert(control.end(), states[i-1].begin(), states[i-1].end());
			}
			//Perform localization step with current measurement and control action
			mySLAM.MCL_step(control, measurements[i],m);
		}
		//Update current belief
		particles = mySLAM.getBelief();
		//Determine state with highest weight
		double maxWeight = 0;
		int index = -1;
		cout <<"Number particles: " << particles.size()<<endl;
		for(int j = 0; j < (int)particles.size(); j++){
			if(particles[j].weight > maxWeight){
				maxWeight = particles[j].weight;
				index = j;
			}
		}
		//Write current state of robot (true and computed) to file
		out <<i<<"\t"<< particles[index].state[0] << "\t"<<particles[index].state[1]<<"\t"<<
				states[i][0]<<"\t"<<states[i][1]<<"\t"<<particles[index].state[2]<<"\t"<<states[i][2]<<endl;
		//Write complete belief to file
		oss << "data/particles_"<<i<<".dat";
		mySLAM.writeBelief(oss.str());
		oss.str("");

	}
	out.close();
	clock_t t_end = clock();
	double elapsed_secs = double(t_end - t_begin) / CLOCKS_PER_SEC;
	cout<<"Time: "<<elapsed_secs<<endl;
}
