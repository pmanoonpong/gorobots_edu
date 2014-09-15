/***************************************************************************
 *   Copyright                                                             *
 *    poramate@physik3.gwdg.de                                             *
 *    fhesse@physik3.gwdg.de                                               *
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

// include simulation environment stuff
#include <ode_robots/simulation.h>
// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
// playground
#include <ode_robots/playground.h>
// simple wiring
#include <selforg/one2onewiring.h>
// the robot
#include <ode_robots/amosII.h>

// include the controller
#include "emptycontroller.h"


// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

// add head file for creating a sphere by Ren ------------
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <iostream>
using namespace std;

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


std::vector<lpzrobots::AbstractObstacle*> obst;
//std::vector<lpzrobots::FixedJoint*> fixator;
// add head file for creating a sphere by Ren ------------

class ThisSim : public lpzrobots::Simulation {
  public:

  ThisSim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    // you can replace color mappings in your own file, see colors/UrbanColorSchema.txt
    // addColorAliasFile("myColorSchema.txt");
    setGroundTexture("Images/whiteground.jpg"); // gets its color from the schema
    //setTitle("centered text");
    //setCaption("right aligned text");
  }

  /**
   * starting function (executed once at the beginning of the simulation loop)
   */
  virtual void start(const lpzrobots::OdeHandle& odeHandle,
      const lpzrobots::OsgHandle& osgHandle,
      lpzrobots::GlobalData& global) {
    // set initial camera position
    setCameraHomePos(
        lpzrobots::Pos(-0.0114359, 6.66848, 0.922832),
        lpzrobots::Pos(178.866, -7.43884, 0));

    // set simulation parameters
    global.odeConfig.setParam("controlinterval", 10);
    global.odeConfig.setParam("simstepsize", 0.01);
    global.odeConfig.setParam("noise", 0.3);

    // add playground
    lpzrobots::Playground* playground
    = new lpzrobots::Playground(odeHandle, osgHandle,
        osg::Vec3(10, 0.2, 0.3));
    playground->setTexture(0,0,lpzrobots::TextureDescr("Images/wall_bw.jpg",-1.5,-3));
    playground->setPosition(osg::Vec3(0,0,.0));
    global.obstacles.push_back(playground);

    //add obstacle
    PassiveBox* b1;
    double length = 3;
    double width = 0.2;
    double height = 1.0;
    bool obstacle_active = false;
    if(obstacle_active)
    {
      b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height /*size*/));
      //b1->setTexture("dusty.rgb");
      b1->setColor(Color(1,0,0));
      //b1->setPosition(osg::Vec3(/*-4.5+*/i*4.5,3+i,0)); // Fixed robot position
      //osg::Matrix pose;
      //      pose.setTrans(osg::Vec3(/*-4.5+*/i*4.5,3+i,0));
      //b1->setPose(osg::Matrix::rotate(0.5*(M_PI/2), 0,0, 1) * osg::Matrix::translate(/*-4.5+*/i*4.5,3+i,0.5) /* pose*/);
      b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0,0, 1) * osg::Matrix::translate(2.0, 0.0,0.0) /* pose*/);
      global.obstacles.push_back(b1);
      lpzrobots::FixedJoint* fixator = new  lpzrobots::FixedJoint(b1->getMainPrimitive(), global.environment);
      fixator->init(odeHandle, osgHandle);
    }


    //----------create a sphere as the target by Ren-----------------------------
    //the first sphere
    lpzrobots::PassiveSphere* s1 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
    s1->setPosition(osg::Vec3(3.0, 0.0, 0.1));
    s1->setTexture("Images/dusty.rgb");
    s1->setColor(lpzrobots::Color(1,0,0));
    obst.push_back(s1);
    global.obstacles.push_back(s1);
    lpzrobots::FixedJoint* fixator1 = new  lpzrobots::FixedJoint(s1->getMainPrimitive(), global.environment);
    fixator1->init(odeHandle, osgHandle);

    //the second sphere
    lpzrobots::PassiveSphere* s2 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
    s2->setPosition(osg::Vec3(0.0, 3.0, 0.1));
    s2->setTexture("Images/dusty.rgb");
    s2->setColor(lpzrobots::Color(0,1,0));
    obst.push_back(s2);
    global.obstacles.push_back(s2);
    lpzrobots::FixedJoint* fixator2 = new  lpzrobots::FixedJoint(s2->getMainPrimitive(), global.environment);
    fixator2->init(odeHandle, osgHandle);

    //the third sphere
    lpzrobots::PassiveSphere* s3 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
    s3->setPosition(osg::Vec3(0.0, -3.0, 0.1));
    s3->setTexture("Images/dusty.rgb");
    s3->setColor(lpzrobots::Color(0,0,1));
    obst.push_back(s3);
    global.obstacles.push_back(s3);
    lpzrobots::FixedJoint* fixator3 = new  lpzrobots::FixedJoint(s3->getMainPrimitive(), global.environment);
    fixator3->init(odeHandle, osgHandle);

    //----------create a sphere as the target by Ren-----------------------------

    // Add amosII robot
    lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
    myAmosIIConf.rubberFeet = true;
    lpzrobots::OdeHandle rodeHandle = odeHandle;
    rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);
    //------------------- Link the sphere to the Goal Sensor by Ren---------------
    for(unsigned int i = 0; i<obst.size(); i++)
    {
      myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
    }
    //------------------- Link the sphere to the Goal Sensor by Ren---------------
    amos = new lpzrobots::AmosII(
        rodeHandle,
        osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
        myAmosIIConf, "AmosII");

    // define the usage of the individual legs
    amos->setLegPosUsage(amos->L0, amos->LEG);
    amos->setLegPosUsage(amos->L1, amos->LEG);
    amos->setLegPosUsage(amos->L2, amos->LEG);
    amos->setLegPosUsage(amos->R0, amos->LEG);
    amos->setLegPosUsage(amos->R1, amos->LEG);
    amos->setLegPosUsage(amos->R2, amos->LEG);

    // put amos a little bit in the air
    amos->place(osg::Matrix::translate(.0, .0, 0.5));

    controller = new EmptyController();
    // create wiring
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

    // create agent and init it with controller, robot and wiring
    lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
    agent->init(controller, amos, wiring);

    // create a fixed joint to hold the robot in the air at the beginning
    robotfixator = new lpzrobots::FixedJoint(
        amos->getMainPrimitive(),
        global.environment);
    robotfixator->init(odeHandle, osgHandle, false);

    // inform global variable over everything that happened:
    global.configs.push_back(amos);
    global.agents.push_back(agent);
    global.configs.push_back(controller);

    std::cout << "\n\n"
        << "################################\n"
        << "#   Press x to free amosII!    #\n"
        << "################################\n"
        << "\n\n" << std::endl;

  }

  /**
   * add own key handling stuff here, just insert some case values
   */
  virtual bool command(const lpzrobots::OdeHandle&,
      const lpzrobots::OsgHandle&,
      lpzrobots::GlobalData& globalData,
      int key,
      bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch (char(key)) {
        case 'x':
          if (robotfixator) {
            std::cout << "dropping robot" << std::endl;
            delete robotfixator;
            robotfixator = NULL;
          }
          break;
        default:
          return false;
          break;
      }
    }
    return false;
  }
  protected:
  lpzrobots::Joint* robotfixator;
  AbstractController* controller;
  lpzrobots::AmosII* amos;
};

int main(int argc, char **argv)
{
  ThisSim sim;
  sim.setGroundTexture("Images/greenground.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}

