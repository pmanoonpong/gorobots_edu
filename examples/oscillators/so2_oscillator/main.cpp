// SO2 oscillator

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>


#include "NonlinearOscillator.h"



#define pi 3.14159265

int main(int argc, char* argv[])
{

  ofstream saveFile1;
  saveFile1.open("ReadSensors1.txt",ios::out);



  /****SO2****/

  double alph = 1.5;//1.5;
  double phi = 0.25;
  double WeightH1_H1;
  double WeightH2_H2;
  double WeightH1_H2;
  double WeightH2_H1;

  double BiasH1;
  double BiasH2;

  double activityH1;
  double activityH2;

  double outputH1;
  double outputH2;


  activityH1 = 0;
  activityH2 = 0;

  outputH1 = 0.01;
  outputH2 = 0.01;



  printf("initial\n");


  do{


    /****SO2*********/

    WeightH1_H1  =  alph*cos(phi);
    WeightH2_H2  =  alph*cos(phi);
    WeightH1_H2  =  alph*sin(phi);
    WeightH2_H1  = -alph*sin(phi);


    BiasH1      = 0.0;
    BiasH2      = 0.0;

    activityH1 = WeightH1_H1*outputH1+WeightH1_H2*outputH2+BiasH1;
    activityH2 = WeightH2_H2*outputH2+WeightH2_H1*outputH1+BiasH2;

    outputH1 = tanh(activityH1);
    outputH2 = tanh(activityH2);


    /****************/



    printf("CPG %f %f\n",outputH1, outputH2);

    saveFile1 <<outputH1 <<" "<<outputH2<<" "<<activityH1<<" "<<activityH2<<" "<<phi<< "   \n" << flush; //SAVE DATA



    // saveFile1 <<F<<"   \n" << flush; //SAVE DATA


    
  }while(1==1);


  return 0;
}





