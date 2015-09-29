/*
 * LocoKit4LegsSimulation.cpp
 *
 *  Created on: Dec 27, 2014
 *      Author: leon
 */

#include "SpringyBotSimulation.h"

namespace lpzrobots {

SpringyBotSimulation::SpringyBotSimulation() {
	setTitle("LocoKit simulation");
	setGroundTexture("whiteground_crosses.jpg");
	simulation_time_seconds = 0.0;
	trial_number = 0;
	agent = NULL;
}

SpringyBotSimulation::~SpringyBotSimulation() {
	// TODO Auto-generated destructor stub
}

bool SpringyBotSimulation::command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down) {
	return false;
}

void SpringyBotSimulation::bindingDescription(osg::ApplicationUsage& au) const {
}

void SpringyBotSimulation::start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) {

	// Configure environment
	setCameraHomePos(Pos(-5, 5, 2),  Pos(-135, -8.5, 0));
	global.odeConfig.setParam("controlinterval", 1);
	global.odeConfig.setParam("gravity", -9.8);

   // Configure simulation
	simulation_time_seconds = 100;
	number_of_runs = 1;
	instantiateAgent(global);

}

void SpringyBotSimulation::addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
	if (globalData.sim_step >= simulation_time)
		simulation_time_reached=true;
}

bool SpringyBotSimulation::restart(const OdeHandle&, const OsgHandle&, GlobalData& global)
{
	if (this->currentCycle == number_of_runs)
		return false;

	if (agent!=0) {
		OdeAgentList::iterator itr = find(global.agents.begin(),global.agents.end(),agent);
		if (itr!=global.agents.end())
		{
			global.agents.erase(itr);
		}
		delete agent;
		agent = 0;
	}

	instantiateAgent(global);

	return true;
}

void SpringyBotSimulation::setSimulationDuration(double seconds) {
	simulation_time = (long)(seconds/0.01);
}

void SpringyBotSimulation::instantiateAgent(GlobalData& global) {
	// Instantiate robot
	LocoKitConf conf = SpringyBot::getDefaultConf();
	robot = new SpringyBot(odeHandle, osgHandle, conf, "LocoKit robot");
	robot->addSensor(std::make_shared<SpeedSensor>(1), Attachment(-1)); // Global speed sensor
	robot->place(Pos(.0, .0, -0.2));

	// Instantiate controller
	controller = new SpringyBotEmptyController("LocoKit Controller");

	// Create the wiring
	auto wiring = new One2OneWiring(new NoNoise());

	// Create Agent
	agent = new OdeAgent(global);
	agent->init(controller, robot, wiring);
	global.agents.push_back(agent);
	global.configs.push_back(agent);

	setSimulationDuration(simulation_time_seconds);
}

} /* namespace lpzrobots */


