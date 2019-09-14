// This example of Feedforward Network with
// Online Backpropagation learning
// for XNOR problem
// By Poramate Manoonpong, 06.12.2013

///////////////////////////////////////////
#include "ann_xnor.h"
#include <string.h>
#include <stdio.h>
#define BPM_ITER	200000 
///////////////////////////////////////////

ANN_XNOR* callann;


int main (int argc, char **argv)
{



  callann = new ANN_XNOR();
  int t;

  /////Initial Neural network///////////////////////


  for (int i=0;i<BPM_ITER;i++) {


    //XNOR sigmoid function training//
    callann->FeedforwardNetwork(0,0,1);
    callann->FeedforwardNetwork(0,1,0);
    callann->FeedforwardNetwork(1,0,0);
    callann->FeedforwardNetwork(1,1,1);


    //XNOR function testing set //
    /*
callann->Run(0,0);
callann->Run(0,1);
callann->Run(1,0);
callann->Run(1,1);
     */

  }


  return 0;
}
