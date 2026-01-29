// Nachstedt T.; Woergoetter, F.; Manoonpong, P. (2012) Adaptive Neural Oscillator with Synaptic Plasticity Enabling Fast Resonance Tuning. International Conference on Artificial Neural Networks (ICANN2012), Lausanne, Switzerland, Part I, LNCS 7552, pp. 451-458
//


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>


#include "NonlinearOscillator.h"



#define pi 3.14159265
#define adaptiveCPGwithPlasticity
//#define adaptiveCPG

int main(int argc, char* argv[])
{



  double x_dot; //dx/dt
  double y_dot; //dx/dt
  double w_dot; //dx/dt
  double y;
  double E;
  double F;
  double delta;
  double q;
  double w;
  double x;
  double TAU;
  double t;
  double temp;
  double g;
  double mu;
  double wd;
  double error;
  int steps;

  double Lr_Hopf;



  x_dot = 0.0; //dx/dt
  y_dot = 0.0; //dx/dt
  w_dot = 0.0; //dx/dt
  y = 0.0;
  F = 0.0;
  delta = 50;
  q = 1;
  w = 35;//pi;
  x = 1.0;
  TAU = 0.01;//0.01;
  t = 0.0;
  g = 0.0;
  wd= 3;//10;//0.5;
  steps = 0;

  Lr_Hopf = 1.5; /// Learning = 15 or 30 not different in speed

  ofstream saveFile1;
  saveFile1.open("ReadSensors1.txt",ios::out);



  /****Timo V1 SO2****/
  double alph = 1.01;//1.5;//1.5;
  double phi = 0.0;//0.25;
  double WeightH1_H1;
  double WeightH2_H2;
  double WeightH1_H2;
  double WeightH2_H1;


  double Weight00;
  double Weight01;
  double Weight10;
  double Weight11;


  double Weight20;
  double Weight02;
  double Weight2F;

  double Weight20_t0;
  double Weight02_t0;
  double Weight2F_t0;


  double A20;
  double A02;
  double A2F;

  double B20;
  double B02;
  double B2F;

  double activity0;
  double activity1;
  double activity2;

  double output0;
  double output1;
  double output2;


  double BiasH1;
  double BiasH2;

  double activityH0;
  double activityH1;

  double outputH0;
  double outputH1;
  double 	intrinsicFreq;

  activityH0 = 0;
  activityH1 = 0;

  outputH0 = 0.01;
  outputH1 = 0.01;


  intrinsicFreq = 0.04;

  phi = intrinsicFreq*(2*pi);//--> intrinsic freq = 0.04 = phi/(2*pi) --> phi = 0.04*(2*pi);




  ///With plasticity
  Weight20 = 0.0;
  Weight02 = 1.0;
  Weight2F = 0.01;

  Weight20_t0 = 0.0;
  Weight02_t0 = 1.0;
  Weight2F_t0 = 0.01;


  activity0 = 0.0;
  activity1 = 0.0;
  activity2 = 0.0;

  output0 = 0.01;
  output1 = 0.01;
  output2 = 0.0;

  A20 = 1;
  A02 = 1;
  A2F = 1;

  B20 = 0.01;
  B02 = 0.01;
  B2F = 0.01;
  F = 0.0;

  printf("initial\n");



  do{


    /****SO2*********/

    
#ifdef adaptiveCPG
    mu = 0.1;
    E = 0.0004;
    wd =0.03;//0.02;//0.03;


    printf("adaptiveCPG\n");


    F = sin(wd*2*pi*t); // P. 53 Timo thesis //2.2*cos(wd*t);


    WeightH1_H1  =  alph*cos(phi);// 1.4;
    WeightH2_H2  =  alph*cos(phi);//1.4;
    WeightH1_H2  =  alph*sin(phi);//input1);%0.4;
    WeightH2_H1  = -alph*sin(phi);//1*input1);%-0.4;


    BiasH1      = 0.0;
    BiasH2      = 0.0;

    activityH0 = alph*cos(phi)*outputH0 + alph*sin(phi)*outputH1+ E*F;
    activityH1 = -alph*sin(phi)*outputH0 + alph*cos(phi)*outputH1;

    //activityH0 = WeightH1_H1*outputH1+WeightH1_H2*outputH2+ E*F;
    //activityH1 = WeightH2_H2*outputH2+WeightH2_H1*outputH1;

    outputH0 = tanh(activityH0);
    outputH1 = tanh(activityH1);

    //Adaptation1
    phi += mu*E*F*outputH1/(sqrt(outputH0*outputH0+outputH1*outputH1));//(freq+0.0131)/0.5176;


    intrinsicFreq = phi/(2*pi);

    //error = freq-wd;


    t= t+1;



    printf("CPG %f %f %f\n",F, outputH0, outputH1);//, Theta_dot, Theta);

    saveFile1 <<F<<" "<<outputH0<<" "<<outputH1<<" "<<activityH0<<" "<<activityH1<<" "<<alph<<" "<<intrinsicFreq<< "   \n" << flush; //SAVE DATA

#endif




#ifdef adaptiveCPGwithPlasticity

    mu = 1;
    wd =0.02;
    //Weight20_old = Weight20;
    //Weight02_old = Weight02;
    //Weight2F_old = Weight2F;


    printf("adaptiveCPGwithPlasticity\n");








    //External Perturbation
    F = 0.2*sin(wd*2*pi*t); // P. 53 Timo thesis //2.2*cos(wd*t);



    //Network
    Weight00  =  alph*cos(phi);
    Weight11  =  alph*cos(phi);
    Weight01  =  alph*sin(phi);
    Weight10  = -alph*sin(phi);

    activity0 = Weight00*output0 + Weight01*output1+Weight02*output2;
    activity1 = Weight10*output0 + Weight11*output1;
    activity2 = Weight20*output0 + Weight2F*F;

    output0 = tanh(activity0);
    output1 = tanh(activity1);
    output2 = tanh(activity2);




    //Adaptation
    Weight20 = Weight20 - A20*output2*output0-B20*(Weight20-Weight20_t0);

    Weight02 = Weight02 - A02*output0*output2-B02*(Weight02-Weight02_t0);

    Weight2F = Weight2F + A2F*output2*F-B2F*(Weight2F-Weight2F_t0);

    phi += mu*Weight02*output2*Weight01*output1;


    intrinsicFreq = phi/(2*pi);

    //error = freq-wd;

    saveFile1 <<F<<" "<<output0<<" "<<output1<<" "<<output2<<" "<<Weight20<<" "<<Weight02<<" "<<Weight2F<<" "<<intrinsicFreq<<" "<<phi<< "   \n" << flush; //SAVE DATA



    t= t+1;



    printf("CPG %f %f %f %f %f\n",F, output0, output1, output2,intrinsicFreq);//, Theta_dot, Theta);


#endif


  }while(1==1);
  
  
  return 0;
}





