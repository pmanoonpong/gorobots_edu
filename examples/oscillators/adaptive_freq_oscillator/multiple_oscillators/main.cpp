//Programmable Central Pattern Generators: an application to biped locomotion control

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>


#include "NonlinearOscillator.h"



#define pi 3.14159265

int main(int argc, char* argv[])
{



  double t;
  double temp;
  double g;
  double error;
  int steps;
  /*
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
   */


  double  P_teach;
  double x_dot[4];
  double x[4];
  double y_dot[4];
  double y[4];
  double w_dot[4];
  double w[4];
  double phi_dot[4];
  double phi[4];
  double alpha_dot[4];
  double alpha[4];
  double theta[4];
  double sgn_x[4];

  double gamma;
  double mu;
  double E;
  double F;
  double T_m;

  double lr;
  double TAU;
  double	Q_learned;



  P_teach = 0.0;

  for(int i=0; i<3; i++)
  {
    x_dot[i] = 0.0;//*
    x[i] = 1.0;//************
    y_dot[i] = 0.0;//*
    y[i] = 0.0;//*
    w_dot[i] = 0.0;//*
    phi_dot[i] = 0.0;//*
    phi[i] = 0.0;//*
    alpha_dot[i] = 0.0;//*
    alpha[i] = 0.0;//*
    theta[i] = 0.0;//*
    sgn_x[i] = 0.0;//*
  }


  //Depend also on initial freq of each oscillator!!!! if initial freq is far from the target... difficult to convert to the target freq!!

  w[0] = 10.0; //********
  w[1] = 20.0; //********
  w[2] = 55.0; //********
  //w[3] = 30.0; //********


  gamma = 8.0;
  mu = 1.0;
  E = 0.9;
  F = 0.0;
  T_m = 2.0;

  lr = 0.5;
  TAU = 0.01;
  Q_learned = 0.0;
  t = 0.0;


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
    //	g  = sqrt(x*x+y*y);

    P_teach = 0.8*sin(15*t)+cos(30*t)-1.4*sin(45*t);//-0.5*cos(60*t);//sin(20*t); 




    t= t+0.01;





    //Oscillator 1
    x_dot[0] = gamma*(mu-(x[0]*x[0]+y[0]*y[0]))*x[0]-w[0]*y[0]+E*F+T_m*sin(theta[0]-phi[0]);
    y_dot[0] = gamma*(mu-(x[0]*x[0]+y[0]*y[0]))*y[0]+w[0]*x[0];
    w_dot[0] = -1*E*F*(y[0]/(sqrt(x[0]*x[0]+y[0]*y[0])));
    alpha_dot[0] = lr*x[0]*F;
    phi_dot[0] = sin((w[0]/w[0])*theta[0]-theta[0]-phi[0]);//********


    x[0] = x[0] + TAU*x_dot[0];
    y[0]= y[0] + TAU*y_dot[0];
    w[0] = w[0] + TAU*w_dot[0];
    alpha[0] = alpha[0]+ TAU*alpha_dot[0];
    phi[0] = phi[0] + TAU*phi_dot[0];

    if(x[0]>0)
      sgn_x[0] = 1.0;
    if(x[0]<0)
      sgn_x[0] = -1.0;
    if(x[0]==0)
      sgn_x[0] = 0.0;


    theta[0] = sgn_x[0]*1/(cos(-1*(y[0]/(sqrt(x[0]*x[0]+y[0]*y[0])))));



    //Oscillator 2
    x_dot[1] = gamma*(mu-(x[1]*x[1]+y[1]*y[1]))*x[1]-w[1]*y[1]+E*F+T_m*sin(theta[1]-phi[1]);
    y_dot[1] = gamma*(mu-(x[1]*x[1]+y[1]*y[1]))*y[1]+w[1]*x[1];
    w_dot[1] = -1*E*F*(y[1]/(sqrt(x[1]*x[1]+y[1]*y[1])));
    alpha_dot[1] = lr*x[1]*F;
    phi_dot[1] = sin((w[1]/w[0])*theta[0]-theta[1]-phi[1]);//********


    x[1] = x[1] + TAU*x_dot[1];
    y[1]= y[1] + TAU*y_dot[1];
    w[1] = w[1] + TAU*w_dot[1];
    alpha[1] = alpha[1]+ TAU*alpha_dot[1];
    phi[1] = phi[1] + TAU*phi_dot[1];

    if(x[1]>0)
      sgn_x[1] = 1;
    if(x[1]<0)
      sgn_x[1] = -1;
    if(x[1]==0)
      sgn_x[1] = 0;


    theta[1] = sgn_x[1]*1/(cos(-1*(y[1]/(sqrt(x[1]*x[1]+y[1]*y[1])))));




    //Oscillator 3
    x_dot[2] = gamma*(mu-(x[2]*x[2]+y[2]*y[2]))*x[2]-w[2]*y[2]+E*F+T_m*sin(theta[2]-phi[2]);
    y_dot[2] = gamma*(mu-(x[2]*x[2]+y[2]*y[2]))*y[2]+w[2]*x[2];
    w_dot[2] = -1*E*F*(y[2]/(sqrt(x[2]*x[2]+y[2]*y[2])));
    alpha_dot[2] = lr*x[2]*F;
    phi_dot[2] = sin((w[2]/w[0])*theta[0]-theta[2]-phi[2]);//********


    x[2] = x[2] + TAU*x_dot[2];
    y[2]= y[2] + TAU*y_dot[2];
    w[2] = w[2] + TAU*w_dot[2];
    alpha[2] = alpha[2]+ TAU*alpha_dot[2];
    phi[2] = phi[2] + TAU*phi_dot[2];

    if(x[2]>0)
      sgn_x[2] = 1;
    if(x[2]<0)
      sgn_x[2] = -1;
    if(x[2]==0)
      sgn_x[2] = 0;


    theta[2] = sgn_x[2]*1/(cos(-1*(y[2]/(sqrt(x[2]*x[2]+y[2]*y[2])))));


    /*


	
	//Oscillator 4
	x_dot[3] = gamma*(mu-(x[3]*x[3]+y[3]*y[3]))*x[3]-w[3]*y[3]+E*F+T_m*sin(theta[3]-phi[3]);
	y_dot[3] = gamma*(mu-(x[3]*x[3]+y[3]*y[3]))*y[3]+w[3]*x[3];
	w_dot[3] = -1*E*F*(y[3]/(sqrt(x[3]*x[3]+y[3]*y[3])));
	alpha_dot[3] = lr*x[3]*F;
	phi_dot[3] = sin((w[3]/w[0])*theta[0]-theta[3]-phi[3]);//********


	x[3] = x[3] + TAU*x_dot[3];
	y[3]= y[3] + TAU*y_dot[3];
	w[3] = w[3] + TAU*w_dot[3];
	alpha[3] = alpha[3]+ TAU*alpha_dot[3];
	phi[3] = phi[3] + TAU*phi_dot[3];

	if(x[3]>0)
	sgn_x[3] = 1;
	if(x[3]<0)
	sgn_x[3] = -1;
	if(x[3]==0)
	sgn_x[3] = 0;


	theta[3] = sgn_x[3]*1/(cos(-1*(y[3]/(sqrt(x[3]*x[3]+y[3]*y[3])))));

     */


    //Output///
    Q_learned = alpha[0]*x[0]+alpha[1]*x[1]+alpha[2]*x[2];//+alpha[3]*x[3];;

    F = P_teach - Q_learned;




    printf("CPG %f %f %f %f %f %f\n",F,Q_learned, P_teach, w[0], w[1], w[2]);//, x[2], x[3]);//, Theta_dot, Theta);

    saveFile1 <<F<<" "<<x[0]<<" "<<x[1]<<" "<<x[2]<<" "<<Q_learned<<" "<<P_teach<<" "<<w[0]<<" "<<w[1]<<" "<<w[2]<< "   \n" << flush; //SAVE DATA


    // saveFile1 <<F<<"   \n" << flush; //SAVE DATA



  }while(1==1);


  return 0;
}





