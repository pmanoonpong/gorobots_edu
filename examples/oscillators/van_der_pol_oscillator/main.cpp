// VanDerPolOscillator.cpp : Defines the entry point for the console application.
//


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>


#include "VanDerPolOscillator.h"

	

int main(int argc, char* argv[])
{
/*
//The van der Pol oscillator is an oscillator with nonlinear damping governed by the second-order differential equation 

x_dot_dot - b(1-x*x)*x_dot+x = 0; d2x/dt2 - b (1 - x*x)*dx/dt + x = a cos(ct)
x = dynamical variable, b>0 is a parameter


This model was proposed by Balthasar van der Pol (1889-1959) 
in 1920 when he was an engineer working for Philips Company (in the Netherlands). 
*/
	
/*
Relaxation Oscillations

//Large value "b" = e.g., 10 = 
 For large values of b the oscillations take on the form of relaxation oscillations.
 We can try to verify the shape of the orbit in phase space using e.g. b=10. 
 (Because of the rapid time dependence over part of the cycle, 
 the numerical integration must use a small time step. 
 If the dynamics is too slow on your computer, reduce b and increase dt. 

  b = 10, TAU = 0.01
  b = 0.05, TAU = 0.2
*/

/*
Small amplitude oscillations

For small values of b (and no driving) the perturbation theory predicts 
an orbit x=2cost, y=-2sint i.e. a simple harmonic motion with a circular orbit in (x,v) 
phase space of radius 2. 
This can be checked by numerical simulation (here for b=0.05. 

*/

/*
Quasiperiodic Motion
Suppose now the Van der Pol oscillator is driven at frequency 1.15 with a strength a=0.32 , 
Other parameters : b =0.2, c =1.15, TAU = 0.2
*/

/*
Frequency Locking
Increasing the nonlinearity tends to increase the tendency of the oscillators to lock. 
So now we change the driving strength to a=0.5 
	
*/
	
	double X;
    double Y;
	double Z;

	double X_dot; //dX/dt
    double Y_dot; //dY/dt
	double Z_dot; //dZ/dt

	double b = 1.0; // 0.05 
	double a = 0.32;
	double c = 1.15;
	double  TAU = 0.02;
	double  TAU2 = 0.3;

    
	ofstream saveFile1;
    saveFile1.open("ReadSensors1.txt",ios::out);

	X = 1.5;
	Y = 0.0;
	Z = 0.0;
    
	X_dot = 0.0;
	Y_dot = 0.0;
	Z_dot = 0.0;

	
   

	//double exppp = exp(-1/Tau_cpg);  // for hip neurons
	//double exppp2= exp(-1/Tau_dat_cpg);  // for hip neurons

	printf("initial\n");





	do{
 


// The Van der Pol oscillator is described by the equations 

// d2x/dt2 - b (1 - x2) dx/dt + x = a cos(ct) 

//In autonomous form with X = x, Y = dx/dt, Z =c t: 



	X_dot  = Y/TAU2;
	Y_dot = (b*(1 - X*X)*Y - X)/TAU2;// + a*cos(Z);


//	Z_dot = c; 


	//Update state variable with Euler first order's method!

    X += TAU*X_dot;
    Y += TAU*Y_dot;
//	Z += TAU*Z_dot;
	
	 

	//////////////////////////////////////


   printf("CPG %f %f %f %f %f %f\n", X, Z, Y, X_dot, Y_dot, Z_dot);

   saveFile1 <<X<<" "<<Y<<" "<<X_dot<< "   \n" << flush; //SAVE DATA 
	
	}while(1==1);


	return 0;
}
