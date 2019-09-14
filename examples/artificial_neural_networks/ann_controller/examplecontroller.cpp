/***************************************************************************
 *   Copyright (C) 2012 by Timo Nachstedt                                  *
 *    nachstedt@physik3.gwdg.de                                            *
 *                                                                         *
 * This is an example controller!                                          *
 *                                                                         *
 * Normally, the controller files should be placed in controller/amosII    *
 *                                                                         *
 **************************************************************************/

#include "examplecontroller.h"
#include <utils/ann-framework/ann.h>
#include <utils/ann-library/so2cpg.h>
#include <utils/ann-library/psn.h>
#include <utils/ann-library/vrn.h>
#include <ode_robots/amosiisensormotordefinition.h>
#include <vector>

/******************************************************************************
 *     Construct your neuronal net
 *****************************************************************************/

class ExampleANN : public ANN {
  public:

    ExampleANN() {
      // create three input neurons
      for (int i=0; i<3; i++) inputNeurons.push_back(addNeuron());

      // create 18 output neurons
      for (int i=0; i<18; i++) outputNeurons.push_back(addNeuron());

      // create the sub networks
      cpg = new SO2CPG();
      psn = new PSN();
      vrnLeft = new VRN();
      vrnRight = new VRN();

      // add the sub networks to this network
      addSubnet(cpg);
      addSubnet(psn);
      addSubnet(vrnLeft);
      addSubnet(vrnRight);

      // connect synapses originating at input neurons
      w(psn->n(0),      inputNeurons[0], -1);
      w(psn->n(1),      inputNeurons[0],  1);
      w(vrnLeft->n(1),  inputNeurons[1],  5);
      w(vrnRight->n(1), inputNeurons[2],  5);


      // create synapses from cpg to psn
      w(psn->n(2), cpg->n(0), 0.5);
      w(psn->n(3), cpg->n(1), 0.5);
      w(psn->n(4), cpg->n(1), 0.5);
      w(psn->n(5), cpg->n(0), 0.5);

      // create synapses from psn to VRN networks
      w(vrnLeft->n(0),  psn->n(11), 1.75);
      w(vrnRight->n(0), psn->n(11), 1.75);

      // create synapses to output neurons
      w(outputNeurons[FL0_m], psn->n(10),  0.5);
      w(outputNeurons[FL1_m], psn->n(10), -0.5);
      w(outputNeurons[FL2_m], psn->n(10),  0.5);
      w(outputNeurons[FR0_m], psn->n(10), -0.5);
      w(outputNeurons[FR1_m], psn->n(10),  0.5);
      w(outputNeurons[FR2_m], psn->n(10), -0.5);
      w(outputNeurons[CL0_m], psn->n(11),  0.8);
      w(outputNeurons[CL1_m], psn->n(11), -0.8);
      w(outputNeurons[CL2_m], psn->n(11),  0.8);
      w(outputNeurons[CR0_m], psn->n(11), -0.8);
      w(outputNeurons[CR1_m], psn->n(11),  0.8);
      w(outputNeurons[CR2_m], psn->n(11), -0.8);
      w(outputNeurons[TL0_m], vrnLeft->n(6),  0.6);
      w(outputNeurons[TL1_m], vrnLeft->n(6), -0.6);
      w(outputNeurons[TL2_m], vrnLeft->n(6),  0.6);
      w(outputNeurons[TR0_m], vrnRight->n(6),  0.6);
      w(outputNeurons[TR1_m], vrnRight->n(6), -0.6);
      w(outputNeurons[TR2_m], vrnRight->n(6),  0.6);

      // initialize cpg
      cpg->setAlpha(2);
      cpg->setPhi(0.6);
      cpg->setOutput(0,1);
    }


    SO2CPG* getCPG() {
      return cpg;
    }

    PSN* getPSN() {
      return psn;
    }

    VRN* getLeftVRN() {
      return vrnLeft;
    }
    VRN* getRightVRN() {
      return vrnRight;
    }

    const double& getOutputNeuronOutput(const int& index)
    {
      return getOutput(outputNeurons[index]);
    }

    void setInputNeuronInput(const int& index, const double& value)
    {
      setInput(inputNeurons[index], value);
    }

  private:
    SO2CPG*  cpg;
    PSN*     psn;
    VRN*     vrnLeft;
    VRN*     vrnRight;
    std::vector<Neuron*> inputNeurons;
    std::vector<Neuron*> outputNeurons;
};



/******************************************************************************
 *     Construct the controller that uses your network
 *****************************************************************************/


ExampleController::ExampleController() : AbstractController("ANN example controller", "1.0") {
  // create neuronal net
  myANN = new ExampleANN;
  // observe neuronal outputs:
  addInspectableValue("cpg.o0", &(myANN->getCPG()->getOutput(0)), "");
  addInspectableValue("psn.o0", &(myANN->getPSN()->getOutput(0)), "");
  addInspectableValue("psn.o1", &(myANN->getPSN()->getOutput(1)), "");
  addInspectableValue("vrnLeft.o1", &(myANN->getLeftVRN()->getOutput(1)), "");
  addInspectableValue("vrnRight.o1", &(myANN->getRightVRN()->getOutput(1)), "");
}

ExampleController::~ExampleController() {
  // delete neuronal net
  delete myANN;
}

void ExampleController::init(int sensornumber, int motornumber, RandGen* randGen) {
  // nothing to do here
}


int ExampleController::getSensorNumber() const {
  return 0;
}

int ExampleController::getMotorNumber() const {
  return 18;
}

void ExampleController::step(const sensor* sensors, int number_sensors,
    motor* motors, int number_motors) {
  // set inputs to your network. You could use sensor values as well.
  myANN->setInputNeuronInput(0,  0);
  myANN->setInputNeuronInput(1,  1);
  myANN->setInputNeuronInput(2, -1);
  // update activities and outputs
  myANN->step();
  // use network output to drive your robot
  for (int i=0; i<18; i++) {
    motors[i] = myANN->getOutputNeuronOutput(i);
  }
}

void ExampleController::stepNoLearning(const sensor*, int number_sensors,
    motor*, int number_motors) {

}

bool ExampleController::store(FILE* f) const {
  return false;
}

bool ExampleController::restore(FILE* f) {
  return false;
}
