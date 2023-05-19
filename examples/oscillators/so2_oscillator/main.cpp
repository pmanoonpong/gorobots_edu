// SO2 oscillator

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>


#include "NonlinearOscillator.h"


#define continuous 
//#define discrete 
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

  double outputH1_dot;
  double outputH2_dot;
 
  double del_t;
  double TAU;  

  activityH1 = 0;
  activityH2 = 0;

  outputH1 = 0.01;
  outputH2 = 0.01;

  outputH1_dot = 0.0;
  outputH1_dot = 0.0;
  
  del_t = 0.01;
  TAU = 0.1;
 
  printf("initial\n");


  do{


#ifdef discrete
    /****SO2*********/
    
    WeightH1_H1  =  alph*cos(phi);
    WeightH2_H2  =  alph*cos(phi);
    WeightH1_H2  =  alph*sin(phi);
    WeightH2_H1  = -alph*sin(phi);


    BiasH1      = 0.0;
    BiasH2      = 0.0;

    //activityH1 = WeightH1_H1*outputH1+WeightH1_H2*outputH2+BiasH1;
    //activityH2 = WeightH2_H2*outputH2+WeightH2_H1*outputH1+BiasH2;

    //outputH1 = tanh(activityH1);
    //outputH2 = tanh(activityH2);

    outputH1 = tanh(WeightH1_H1*outputH1+WeightH1_H2*outputH2+BiasH1);
    outputH2 = tanh(WeightH2_H2*outputH2+WeightH2_H1*outputH1+BiasH2);

    /****************/
#endif

#ifdef continuous
    /****SO2_ cont. time*********/

    WeightH1_H1  =  alph*cos(phi);
    WeightH2_H2  =  alph*cos(phi);
    WeightH1_H2  =  alph*sin(phi);
    WeightH2_H1  = -alph*sin(phi);


    BiasH1      = 0.0;
    BiasH2      = 0.0;

    outputH1_dot = (-outputH1+tanh(WeightH1_H1*outputH1+WeightH1_H2*outputH2+BiasH1))/TAU;
    outputH2_dot = (-outputH2+tanh(WeightH2_H2*outputH2+WeightH2_H1*outputH1+BiasH2))/TAU;

    outputH1 += del_t*outputH1_dot;
    outputH2 += del_t*outputH2_dot;



    /****************/
#endif


    printf("CPG %f %f\n",outputH1, outputH2);

    saveFile1 <<outputH1 <<" "<<outputH2<<" "<<outputH1_dot<<" "<<outputH2_dot<<" "<<phi<< "   \n" << flush; //SAVE DATA
    //saveFile1 <<outputH1 <<" "<<outputH2<<" "<<activityH1<<" "<<activityH2<<" "<<phi<< "   \n" << flush; //SAVE DATA



    // saveFile1 <<F<<"   \n" << flush; //SAVE DATA


    
  }while(1==1);


  return 0;
}





