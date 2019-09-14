/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 *   $Log: main.cpp,v $                                                    *
 *                                                                         *
 ***************************************************************************/


#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/complexplayground.h>
#include <ode_robots/terrainground.h>
#include <ode_robots/passivebox.h>

#include <selforg/invertnchannelcontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

#include <ode_robots/amosII.h>
//#include <ode_robots/amosIIconf.h>
#include "amosIIcontrol.h"

#include <selforg/trackrobots.h>






// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

class ThisSim : public Simulation {
public:

  AbstractController* controller;
  OdeRobot* robot;
  Playground* playground;


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    // set initial camera position
    setCameraHomePos(Pos(-0.0114359, 6.66848, 0.922832),  Pos(178.866, -7.43884, 0));


	global.odeConfig.noise= 0.05;//0.05;//0.05;//0.05;

    // set simulation parameters
    global.odeConfig.setParam("controlinterval", 20 /*Set update frequency of the simulation*/);//was 10
    global.odeConfig.setParam("simstepsize", 0.005); //was 0.01 (martin)
   //Update frequency of simulation = 20*0.005 = 0.1 s ; 10 Hz

    
    /*******  Begin Modify Environment *******/

    // Possibility to add Playground as boundary:
    // to activate change false to true
    bool use_playground = false;
    if (use_playground){
    	playground = new Playground(odeHandle, osgHandle, osg::Vec3(8, 0.2, 1), 1);
    	playground->setColor(Color(0,0,0,0.8));
    	//playground->setGroundColor(Color(2,2,2,1));
    	playground->setPosition(osg::Vec3(0,0,0.05)); // place and generate playground
    	global.obstacles.push_back(playground);
    }


    // Possibility to add obstacles
    double scale_size = 10;//10;
    int bars = 0;
    for(int i=0; i< bars; i++){

      PassiveBox* b = new PassiveBox(odeHandle, osgHandle.changeColor(Color(0.,0.6,0.)),
				     osg::Vec3(0.5 /*width*/,0.5 /*length*/,.08+i*0 /*height*/), 0.0); // size
      b->setPosition(osg::Vec3(/*0.5*scale_size*/0.6+i*2,0,0));
      global.obstacles.push_back(b);    
      // With fixed joints you can fix things in the environment
//      Joint* fixator;
//      Primitive* p = b->getMainPrimitive(); // set b or your object here
//      fixator = new FixedJoint(p, global.environment);
//      fixator->init(odeHandle, osgHandle);

    }



    // add one box
    // to activate change false to true
    bool use_box = 0;
    if (use_box){
    	double angle = 0.0;
    	double height = 0.06;
        Substance BoxSubstance(2.0,0.001,100.0,0.4); //(roughness,slip,hardness,elasticity)
		OdeHandle oodeHandle = odeHandle;
		oodeHandle.substance = BoxSubstance;//.toRubber(boxsubparam);//toMetal(boxsubparam);
		PassiveBox* s1 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,100 /*width*/,height /*height*/), 0.0);
		s1->setTexture("../../../../osg/data/Images/chess.rgb");
		s1->setPose(osg::Matrix::rotate(-M_PI/180.0 * angle,0,1,0) * osg::Matrix::translate(0.9,0,height/2));
		global.obstacles.push_back(s1);
    }
    // add a small street through rough terrain
    // to activate change false to true
    bool use_channel = false;
    if (use_channel){
    	double angle = 0.0;
		OdeHandle oodeHandle = odeHandle;
		oodeHandle.createNewSimpleSpace(oodeHandle.space,true);
		TerrainGround* terrainground =
		new TerrainGround(oodeHandle, osgHandle.changeColor(Color(83.0/255.0,48.0/255.0,0.0/255.0)),
			"rough1.ppm" /*rough6.ppm*/,"", 10, 10, 0.01/*0.15, height*/);
		terrainground->setPose(osg::Matrix::translate(0, 0, 0.01));
		global.obstacles.push_back(terrainground);
        Substance BoxSubstance(2.0,0.001,100.0,0.4); //(roughness,slip,hardness,elasticity)
		oodeHandle.substance = BoxSubstance;//.toRubber(boxsubparam);//toMetal(boxsubparam);
		PassiveBox* s1 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(10,10,1), 0.0);
		s1->setTexture("../../lpzrobots/ode_robots/osg/data/Images/chess.rgb");
		s1->setPose(/*osg::Matrix::rotate(-M_PI/180.0 * angle,0,1,0) * */osg::Matrix::translate(0,5.3,0));
		global.obstacles.push_back(s1);
		PassiveBox* s2 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(10,10,1), 0.0);
		s2->setTexture("../../lpzrobots/ode_robots/osg/data/Images/chess.rgb");
		s2->setPose(/*osg::Matrix::rotate(-M_PI/180.0 * angle,0,1,0) * */osg::Matrix::translate(0,-5.3,0));
		global.obstacles.push_back(s2);
//		oodeHandle.addIgnoredPair(s1->getMainPrimitive(),s2->getMainPrimitive());

//		oodeHandle.addIgnoredPair(terrainground->getMainPrimitive(),s1->getMainPrimitive());
//		oodeHandle.addIgnoredPair(terrainground->getMainPrimitive(),s2->getMainPrimitive());
    }

/*    // With fixed joints you can fix things in the environment
    Joint* fixator;
    Primitive* p = s1->getMainPrimitive();
    fixator = new FixedJoint(p, global.environment);
    fixator->init(odeHandle, osgHandle);
*/


// add rough terrain
    bool use_rough_ground=1;
    if (use_rough_ground){

    	//**************Change Material substance*********//
    	Substance roughterrainSubstance(1.0,0.0,/*100.0 friction*/500.0,0.0); //(roughness,slip,hardness,elasticity)
    	OdeHandle oodeHandle = odeHandle;
    	oodeHandle.substance = roughterrainSubstance;
    	//**************Change Material substance*********//

    	TerrainGround* terrainground =
    			new TerrainGround(oodeHandle, osgHandle.changeColor(Color(83.0/255.0,48.0/255.0,0.0/255.0)),
    					/*"obstacles.ppm"*/"rough6.ppm" /*rough1.ppm*//*"1.ppm"*/,"", /*10*/3, /*10*/3, 0.05/*0.1 (without BJ), 0.13 (with BJ), 0.03 (ren) actual ob height, 0.065 m*//*0.15 m*/ /* //Changing terrain height*/);
    	terrainground->setPose(osg::Matrix::translate(0, 0, 0.01));
    	global.obstacles.push_back(terrainground);
    }

    /*******  End Modify Environment *******/
      
      

	/******* Add robot A M O S II  *********/
//Adding feet
    //AmosIIConf myAmosIIConf = AmosII::getDefaultConf(1.0,1,1);
    //AmosIIConf myAmosIIConf = AmosII::getDefaultConf(5.0/*5.0 size*/,1 /*_useShoulder*/,1/*activated feet*/,1/*Backbone*/);
//Not adding feet
   lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf(1.0/*size*/,1 /*_useShoulder*/,1/*activated feet*/,1/*Backbone*/);
    //AmosIIConf myAmosIIConf = AmosII::getFranksDefaultConf(5.0/*size*/,1 /*_useShoulder*/,1/*activated feet*/,0/*Backbone*/);


//	AmosIIConf myAmosIIConf = AmosII::getExperimentConf();

	//myAmosIIConf.coxaPower = 2.5;
	//myAmosIIConf.coxaSpeed   = 10;

	//Robot physical property
	OdeHandle rodeHandle = odeHandle;

	//Substance RobotSubstance(3.0,0.0,50.0,0.8);//(roughness,slip,hardness,elasticity)
	Substance RobotSubstance(3.0,0.0,50.0,0.8);//(roughness,slip,hardness,elasticity)
	rodeHandle.substance = RobotSubstance;

	//Creating Robot & its color
	robot = new AmosII(rodeHandle, osgHandle.changeColor(Color(0.8,0.75,1)),
			  myAmosIIConf, "AmosII");

   //Initial position of the robot

	// upside down
	// robot ->place(osg::Matrix::rotate(M_PI,1,0,0)*osg::Matrix::translate(0,0,3));

	// normal position
	robot->place(osg::Matrix::rotate(M_PI/2,0,0,1) * osg::Matrix::translate(.0,.0,.3));
	global.configs.push_back(robot);  // add robot to configurables of simulation

	// Homemokinetic controller
	//AbstractController* controller = new InvertNChannelController(20);

	// Standard Tripod controller
	AbstractController* controller = new AmosIIControl();

	// create wiring between robot and controller (sensor <--> motor)
	One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
	OdeAgent* agent = new OdeAgent(global);

	// init agent with controller, robot and wiring
	agent->init(controller, robot, wiring);



    // Possibility to add tracking for robot
    bool track= false;
		if(track) agent->setTrackOptions(TrackRobot(true,false,false, true, "")); // Display trace
		//if(track) agent->setTrackOptions(TrackRobot(false,false,false, false, ""));



//    TrackRobot* track2 = new TrackRobot(/*bool trackPos*/false,
//               /*bool trackSpeed*/false,
//               /*bool trackOrientation*/false,
//               /*bool displayTrace*/true //,
//               /*const char *scene="", int interval=1*/);
//   agent2->setTrackOptions(*track2);



    //add agent to agents
    global.agents.push_back(agent);
    //add controller to configurables
    global.configs.push_back(controller);

    showParams(global.configs);

  }


  /**************************Reset Function***************************************************************/
  virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
      {
  	  std::cout << "\n begin restart " << currentCycle << "\n";
  	  // We would like to have 10 runs!
  	  // after it we must clean all and return false because we don't want a new restart
  	if (this->currentCycle == 2){

  	  //clean robots
  	  while (global.agents.size() > 0)
  		{
  		  OdeAgent* agent = *global.agents.begin();
  		  AbstractController* controller = agent->getController();
  		  OdeRobot* robot = agent->getRobot();
  		  AbstractWiring* wiring = agent->getWiring();

  		  global.configs.erase(std::find(global.configs.begin(),
  			  global.configs.end(), controller));
  		  delete controller;

  		  delete robot;
  		  delete wiring;

  		  delete (agent);
  		  global.agents.erase(global.agents.begin());
  		}

  	  // clean the playgrounds
  	  while (global.obstacles.size() > 0)
  		{
  		  std::vector<AbstractObstacle*>::iterator iter =
  			  global.obstacles.begin();
  		  delete (*iter);
  		  global.obstacles.erase(iter);
  		}
  	  std::cout << "end.";
  	  return false; // don't restart, just quit
  	}



  	// Now we must delete all robots and agents from the simulation and create new robots and agents.
  	while (global.agents.size() > 0)
  	{
  //	  std::cout << "\n MAIN WHILE LOOP" << currentCycle << "\n";
  	  OdeAgent* agent = *global.agents.begin();
  	  AbstractController* controller = agent->getController();
  	  OdeRobot* robot = agent->getRobot();
  	  AbstractWiring* wiring = agent->getWiring();

  	  global.configs.erase(std::find(global.configs.begin(),
  		  global.configs.end(), controller));

  	  delete controller;
  	  //this calls destroy:
  	  delete robot;

  	  delete wiring;

  	  delete (agent);

  	  global.agents.erase(global.agents.begin());
  //	  std::cout << "\n END OF MAIN WHILE LOOP" << currentCycle << "\n";

  	}



  	// add robot
  	/*******  A M O S II  *********/
  	Substance RobotSubstance(3.0,0.0,50.0,0.8);


  	lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf();
//  AmosIIConf myAmosIIConf = AmosII::getExperimentConf();

  	//New configuration after Reset Function!!!!
	myAmosIIConf.fLegTrunkAngleH = 0.15;
	myAmosIIConf.mLegTrunkAngleH = 0.15;
	myAmosIIConf.rLegTrunkAngleH = 0.15;

  	OdeHandle rodeHandle = odeHandle;
  	rodeHandle.substance = RobotSubstance;
  	robot = new AmosII(rodeHandle, osgHandle.changeColor(Color(1,1,1)),
  			  myAmosIIConf, "AmosII");

  	// normal position
  	robot->place(/*osg::Matrix::rotate(2*M_PI/10.0 * currentCycle,0,0,1) **/ osg::Matrix::translate(.3,.0,.1));
  	global.configs.push_back(robot);  // add robot to configurables of simulation

	// Homemokinetic controller
	//AbstractController* controller = new InvertNChannelController(20);

	// Standard Tripod controller
	AbstractController* controller = new AmosIIControl();

  	// create wiring and agent
  	One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
  	OdeAgent* agent = new OdeAgent(global);

  	// init agent with controller, robot and wiring
  	agent->init(controller, robot, wiring);

  	// Possibility to add tracking for robot
  	bool track= 1;
  	if(track) agent->setTrackOptions(TrackRobot(true,false,false, false, ""));

  	//add agent to agents
  	global.agents.push_back(agent);
  	//add controller to configurables
  	global.configs.push_back(controller);

  	//for some unknown reason the following crashes the simulation after a few cycles
  //	showParams(global.configs);
  	  std::cout << "\n end restart " << currentCycle << "\n";
  	// restart!
  	return true;
      }
  /****************************************************************************************************/





  /** optional additional callback function which is called every simulation step.
      Called between physical simulation step and drawing.
      @param draw indicates that objects are drawn in this timestep
      @param pause always false (only called of simulation is running)
      @param control indicates that robots have been controlled this timestep
   */
  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    // for demonstration: set simsteps for one cycle to 60.000/currentCycle (10min/currentCycle)
    // if simulation_time_reached is set to true, the simulation cycle is finished


//	/***********************Reset Function*********************************************/
//	if (globalData.sim_step>=(0.5*60.0*200.000)) ///this->currentCycle)) 60*60.0*200.000
//    {
//         simulation_time_reached=true;
//    }
//	/***********************************************************************************/



  }

//  // add own key handling stuff here, just insert some case values
//  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
//  {
//    if (down) { // only when key is pressed, not when released
//      switch ( (char) key )
//	{
//	default:
//	  return false;
//	  break;
//	}
//    }
//    return false;
//  }



};


int main (int argc, char **argv)
{
  ThisSim sim;
  sim.setGroundTexture("green-artificial-grass-texture.jpg");//green_velour_wb.rgb
  sim.setCaption("lpzrobots Simulator (AMOS II)        Goldschmidt 2012");
  return sim.run(argc, argv) ? 0 :  1;
}

 
 
