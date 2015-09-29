// Using nonlinear oscillators to control the locomotion of a simulated biped robot
//


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>


#include "NonlinearOscillator.h"



#define pi 3.14159265

int main(int argc, char* argv[])
{



  double x_dot; //dx/dt
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
  double temp;

  ofstream saveFile1;
  saveFile1.open("ReadSensors1.txt",ios::out);


  x_dot = 0.0;
  v_dot = 0.0;
  x_bar_dot = 0.0;

  x = 0.6;
  v = 0.6;
  x_bar = 0.6;

  x_0 = 1.0;

  printf("initial\n");


  do{





    //Version 1 no control of AMPLITUDE, simple  a harmonic oscillator with a period of 2piT


    v_dot = -x/T;

    x_dot = v/T;


    //Update state variable with Euler first order's method!
    
    v += TAU*v_dot;
    x += TAU*x_dot;



    /*
//The parameter E represents the desired energy.
//The expression x2 + v2 represents the actual energy of the oscillator 
//(x2 approximates the oscillator's potential
// energy and v2 its kinematic energy). 

//In other words x2 + v2 - E represents what we
//could call the energy error of the system. So the biggest "ALPHA" is, 
//the fastest the oscillator will reach its limit cycle.


//Center at 0 Version 2 with control AMPLITUDE
	temp = (x*x+v*v-E)/(x*x+v*v);

	v_dot = ((-Alpha*temp*v)-x)/T;

	x_dot = v/T;


	//Update state variable with Euler first order's method!

    v += TAU*v_dot;
    x += TAU*x_dot;


	//////////////////////////////////////
     */



    /*
 
//Center at X_0 // Version 3 with control and setting center 

	x_bar = x-x_0;

	temp = (x_bar*x_bar+v*v-E)/(x_bar*x_bar+v*v);

	v_dot = ((-Alpha*temp*v)-x_bar)/T; 

	x_dot = v/T;


	//Update state variable with Euler first order's method!

    v += TAU*v_dot;
	x += TAU*x_dot;

     */




    printf("CPG %f %f %f %f\n",v, x_bar, v_dot, x_dot);//, Theta_dot, Theta);

    saveFile1 <<v<<" "<<x<<" "<<v_dot<<" "<<x_dot<< "   \n" << flush; //SAVE DATA

  }while(1==1);


  return 0;
}





