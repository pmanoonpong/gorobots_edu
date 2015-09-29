//An Analog Neural Oscillator Circuit for Locomotion Controller in Quadruped Walking Robot

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>


#include "WilsonOscillator.h"

	

int main(int argc, char* argv[])
{

//	dui/dt = -u+f_mu(A*u-C*v+su);
//	dvi/dt = -v+f_mu(B*u-D*v+sv);


	double u_dot; //du/dt
    double v_dot; //dv/dt
	
	double u; //du/dt
    double v; //dv/dt
	
 // Show Amari-Hopfield model
	double A = 10.0; 
	double B = 5.0;
	double C = 10.0;
	double D = 0.0;


	double f_u;  
	double f_v;
	double Su = 0.0; // External input to u
	double Sv = -2.5; // External input to v

   /*
	//Show The stable equilibrium:
	double A = 2.5; 
	double B = 2.5;
	double C = 2.5;
	double D = 0.0;


	double f_u;  
	double f_v;
	double Su = 0.0; // External input to u
	double Sv = -1.25; // External input to v

*/


    double TAU = 0.01;
 
	double mu = 1.0;

	double tanh_u;
	double tanh_v;

	tanh_u = 0.0;
	tanh_v = 0.0;
    
	ofstream saveFile1;
    saveFile1.open("ReadSensors1.txt",ios::out);

    
	u_dot = 0.0;
	v_dot = 0.0;
	
	u = 0.0;
	v = 0.0;

	f_u = 0.0;
	f_v = 0.0;

	printf("initial\n");
   

	double Theta_dot = 0.0;
	
	double Theta = 0.0;
	double Tc = 1.2;




	do{
 
//	Theta_dot = 2*3.1416/Tc;




	u_dot = -u + A*f_u - C*f_v + Su;

 	v_dot = -v + B*f_u - D*f_v + Sv;

    tanh_u = (2./(1.+exp(-2.*(mu*u)))-1.0);
    tanh_v = (2./(1.+exp(-2.*(mu*v)))-1.0);
   
	f_u = (1+tanh(mu*u))/2;
	f_v = (1+tanh(mu*v))/2;


	//Update state variable with Euler first order's method!

    u += TAU*u_dot;
    v += TAU*v_dot;
	
		Theta += TAU*Theta_dot;
	 

	//////////////////////////////////////


   printf("CPG %f %f %f %f %f %f\n", v, u, v_dot, u_dot);//, Theta_dot, Theta);

   saveFile1 <<v<<" "<<u<<" "<<v_dot<< "   \n" << flush; //SAVE DATA 
	
	}while(1==1);


	return 0;
}





