// Adaptive Frequency Oscillator; From Dynamic Hebbian Learning for Oscillators toAdaptive Central Pattern Generators
//


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>


#include "NonlinearOscillator.h"



#define pi 3.14159265

int main(int argc, char* argv[])
{



  /*	double x_dot; //dx/dt
    double v_dot; //dv/dt
	double x_bar_dot;
	double x_0;

	double x; 
    double v; 
	double x_bar;

	double E = 0.1; // Amplitude = 2 * square root (E)
	double T = 1/pi;//1/(2*pi);
	double Alpha = 2.0;//1.0;

    double TAU = 0.001; //100 ms
		x_dot = 0.0;
	v_dot = 0.0;
	x_bar_dot = 0.0;
	
	x = 0.6;
	v = 0.6;
    x_bar = 0.6;

	x_0 = 1.0;
   */





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
  E = 0.5;//0.9;//0.5;//0.3;
  F = 0.0;
  delta = 50;
  q = 1;
  w = 35;//pi;
  x = 1.0;
  TAU = 0.01;//0.01;
  t = 0.0;
  g = 0.0;
  mu = 1.0;
  wd= 30;
  steps = 0;

  Lr_Hopf = 1.5; /// Learning = 15 or 30 not different in speed

  ofstream saveFile1;
  saveFile1.open("ReadSensors1.txt",ios::out);




  printf("initial\n");


  do{


    
    /* //Rayleigh oscillator
	g  = sqrt(x*x+y*y);

    F = sin(20*t); 

	x_dot = y + E*F;
	y_dot = delta*(1-q*y*y)*y-w*w*x;
	w_dot = E*F*(y/(sqrt(x*x+y*y)));

	x = x + TAU*x_dot;
	y = y + TAU*y_dot;
	w = w + TAU*w_dot;
     */



    /*	 steps++;
     if(steps >= 200000)
	 {
	 E = 0;
	 w = (int)(w);
	 }
     */

    //Hopf oscillator
    g  = sqrt(x*x+y*y);

    F = 2.2*cos(wd*t);//sin(20*t); 

    x_dot = (mu-(x*x+y*y))*x-w*y+E*F;
    y_dot = (mu-(x*x+y*y))*y+w*x;


    w_dot = -1*E*F*(y/(sqrt(x*x+y*y)));

    x = x + TAU*x_dot;
    y = y + TAU*y_dot;
    w = w + TAU*w_dot;

    //	acos(2);

    error = w-wd;


    t= t+0.01;


    printf("CPG %f %f %f %f %f\n",F, w,  error,E, steps);//, Theta_dot, Theta);

    saveFile1 <<F<<" "<<error<<" "<<y<<" "<<x<<" "<<w<<" "<<wd<<" "<<E<<" "<<steps<< "   \n" << flush; //SAVE DATA


    // saveFile1 <<F<<"   \n" << flush; //SAVE DATA



  }while(1==1);


  return 0;
}





