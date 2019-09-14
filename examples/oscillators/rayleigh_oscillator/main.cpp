// VanDerPolOscillator.cpp : Defines the entry point for the console application.
//


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>


#include "RayleighOscillator.h"

	

int main(int argc, char* argv[])
{

	
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
 


// The Rayleigh oscillator is described by the equations

// d2x/dt2 - b (1 - x_dot2) dx/dt + x = a cos(ct)

//In autonomous form with X = x, Y = dx/dt, Z =c t: 



	X_dot  = Y/TAU2;
	Y_dot = (b*(1 - X_dot*X_dot)*Y - X)/TAU2;// + a*cos(Z);


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
