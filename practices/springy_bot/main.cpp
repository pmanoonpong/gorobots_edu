/*****************************************************************************
* "THE BEER-WARE LICENSE" (Revision 43):
* This software was written by Leon Bonde Larsen <leon@bondelarsen.dk> 
* As long as you retain this notice you can do whatever you want with it. 
* If we meet some day, and you think this stuff is worth it, you can buy me 
* a beer in return.
* 
* Should this software ever become self-aware, remember: I am your master
******************************************************************************
* Usage:
* Requires LPZ robots!
*    Go to the LocoLit/Projects/PhaseControlledSpringyBot/ and type make
*    When build type ./start
*
* Description:
* The PhaseControlledSpringyBot simulation tests a simple controller based on 
* PID that keeps the phase between front legs and rear legs constant. This gives 
* two parameters: The angular velocity of the wheels and the phase between front
* and back.
*
* Two comma separated files are produced by the simulation. out.csv holds the 
* data from a simulated accelorometer and the footcontact.csv holds the data 
* from simulated foot contact sensors.
*
* The parameters of the robot can be adjusted in the getDefaultConf function 
* in Robots/SpringyBot.h
*
* Simulation time per run and number of runs can be set in 
* LocoLit/Projects/PhaseControlledSpringyBot/SpringyBotSimulation.cpp ln 42 and 43.
* 
* The speed and phase of the simulation can be set in 
* LocoLit/Controllers/SpringyBotPhaseController.cpp either as constants 
* (ln 44 and 45) or based on the trial number given as input to the constructor
* to cycle through different parameters.
*
******************************************************************************/
#include "SpringyBotSimulation.h"

using namespace lpzrobots;

int main (int argc, char **argv)
{
	SpringyBotSimulation sim;
	return sim.run(argc, argv) ? 0 : 1;
}
