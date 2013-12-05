/*----------------------------------------------------------------------
    This file contains a simulation of the cart and pole dynamic system and 
 a procedure for learning to balance the pole.  Both are described in 
 Barto, Sutton, and Anderson, "Neuronlike Adaptive Elements That Can Solve
 Difficult Learning Control Problems," IEEE Trans. Syst., Man, Cybern.,
 Vol. SMC-13, pp. 834--846, Sept.--Oct. 1983, and in Sutton, "Temporal
 Aspects of Credit Assignment in Reinforcement Learning", PhD
 Dissertation, Department of Computer and Information Science, University
 of Massachusetts, Amherst, 1984.  The following routines are included:

       main:              controls simulation interations and implements 
                          the learning system.

       cart_and_pole:     the cart and pole dynamics; given action and
                          current state, estimates next state

       get_box:           The cart-pole's state space is divided into 162
                          boxes.  get_box returns the index of the box into
                          which the current state appears.

 These routines were written by Rich Sutton and Chuck Anderson.  Claude Sammut 
 translated parts from Fortran to C.  Please address correspondence to
 sutton@gte.com or anderson@cs.colostate.edu
---------------------------------------
Changes:
  1/93: A bug was found and fixed in the state -> box mapping which resulted 
        in array addressing outside the range of the array.  It's amazing this
        program worked at all before this bug was fixed.  -RSS
----------------------------------------------------------------------*/

#include <math.h> 
#include <stdio.h>  /* SAVE DATA */ 
#include <stdlib.h>  /* RANDOM_MAX */ 
#include <time.h>  /* Time */ 

#define min(x, y)               ((x <= y) ? x : y)
#define max(x, y)	        ((x >= y) ? x : y)
#define prob_push_right(s)      (1.0 / (1.0 + exp(-max(-50.0, min(s, 50.0)))))
#define random                  ((float) rand() / (float)((1 << 31) - 1))

#define N_BOXES         162         /* Number of disjoint boxes of state space. */
#define ALPHA		0.8//1000        /* Learning rate for action weights, w. */
#define BETA		0.5         /* Learning rate for critic weights, v. */
#define GAMMA		0.95        /* Discount factor for critic. */
#define LAMBDAw		0.9         /* Decay rate for w eligibility trace. */
#define LAMBDAv		0.8         /* Decay rate for v eligibility trace. */

#define  LAMBDAw_con    0.0//0.9        // cont. e trace of actor


#define MAX_FAILURES     100000         /* Termination criterion. */
#define MAX_STEPS        100000

#define RandomWeights
#define HIGH_w_x 	1.0
#define LOW_w_x		0.0
//#define SCAN

typedef float vector[N_BOXES];

main()
{
  
  float  Initial_x_reset, Initial_Theta_reset;



  /********SAVE DATA****************************/
  FILE *finputs_outputs;
  finputs_outputs = fopen("Inputs_Outputs", "w");

  FILE *finputs_outputs2;
  finputs_outputs2 = fopen("Inputs_Outputs2", "w");

  FILE *finputs_outputs3;
  finputs_outputs3 = fopen("Inputs_Outputs3", "w");

  FILE *finputs_outputs4;
  finputs_outputs4 = fopen("Inputs_Outputs4", "w");


  FILE *finputs_outputs5;
  finputs_outputs5 = fopen("Inputs_Outputs5", "w");

  FILE *finputs_outputs6;
  finputs_outputs6 = fopen("Inputs_Outputs6", "w");

  FILE *fWeightX;
  fWeightX = fopen("WeightX", "w");


  FILE *fWeightTheta;
  fWeightTheta = fopen("WeightTheta", "w");

  FILE *fWeightXdot;
  fWeightXdot = fopen("WeightXdot", "w");

  FILE *fWeightThetadot;
  fWeightThetadot = fopen("WeightThetadot", "w");



  FILE *fWeightX2;
  fWeightX2 = fopen("WeightX2", "w");


  FILE *fWeightTheta2;
  fWeightTheta2 = fopen("WeightTheta2", "w");

  FILE *fWeightXdot2;
  fWeightXdot2 = fopen("WeightXdot2", "w");

  FILE *fWeightThetadot2;
  fWeightThetadot2 = fopen("WeightThetadot2", "w");


  FILE *fWeightX3;
  fWeightX3 = fopen("WeightX3", "w");


  FILE *fWeightTheta3;
  fWeightTheta3 = fopen("WeightTheta3", "w");

  FILE *fWeightXdot3;
  fWeightXdot3 = fopen("WeightXdot3", "w");

  FILE *fWeightThetadot3;
  fWeightThetadot3 = fopen("WeightThetadot3", "w");


  FILE *fWeightX4;
  fWeightX4 = fopen("WeightX4", "w");


  FILE *fWeightTheta4;
  fWeightTheta4 = fopen("WeightTheta4", "w");

  FILE *fWeightXdot4;
  fWeightXdot4 = fopen("WeightXdot4", "w");

  FILE *fWeightThetadot4;
  fWeightThetadot4 = fopen("WeightThetado4", "w");


  FILE *fWeightX5;
  fWeightX5 = fopen("WeightX5", "w");


  FILE *fWeightTheta5;
  fWeightTheta5 = fopen("WeightTheta5", "w");

  FILE *fWeightXdot5;
  fWeightXdot5 = fopen("WeightXdot5", "w");

  FILE *fWeightThetadot5;
  fWeightThetadot5 = fopen("WeightThetadot5", "w");


  FILE *fWeightX6;
  fWeightX6 = fopen("WeightX6", "w");


  FILE *fWeightTheta6;
  fWeightTheta6 = fopen("WeightTheta6", "w");

  FILE *fWeightXdot6;
  fWeightXdot6 = fopen("WeightXdot6", "w");

  FILE *fWeightThetadot6;
  fWeightThetadot6 = fopen("WeightThetadot6", "w");
  /********************************************/



  /*****Inputs*********************************/

  float maxX = 2.4;
  float minX = -2.4;

  float maxtheta = 0.2094384;
  float mintheta = -0.2094384;

  float maxX_dot = 3.5;
  float minX_dot = -3.5;

  float maxtheta_dot = 2.0;
  float mintheta_dot = -2.0;

  float x_com = 0.0, theta_com = 0.0, x_dot_com = 0.0, Theta_dot_com = 0.0 ;
  /********************************************/


  float   weightActual_x_RL = 0.0,
      weightActual_theta_RL = 0.0,
      weightActual_x_dot_RL = 0.0,
      weightActual_theta_dot_RL = 0.0;

  float   e_x_RL_old =  0.0,
      e_theta_RL_old = 0.0,
      e_x_dot_RL_old = 0.0,
      e_theta_dot_old = 0.0;

  float e_x_RL = 0.0,
      e_theta_RL = 0.0,
      e_x_dot_RL = 0.0,
      e_theta_dot_RL = 0.0;




  //--------------------------------------ACTOR--------------------------------------------------//
  // Call noise function
  //#### (1)
  double original_noise = 0.0;

  // USED NOISE from original noise * prediction value of critic (p)
  double   noise_p = 0.0;


  //#### (2)
  /////// OUTPUT of actor = linear combination from 4 inputs + noise*p//////////////////////
  double output_RL = 0.0;
  float steps_noise  = 1.0;
  int Bigloop;



  ///LOOP
  for(Bigloop = 0; Bigloop<6; Bigloop++) // MAXIMUM <20 rounds
  {



#ifdef SCAN
    for(Initial_x_reset = -2.4; Initial_x_reset < 2.4; Initial_x_reset = Initial_x_reset+0.1)

    {

      for(Initial_Theta_reset = -0.2094395; Initial_Theta_reset < 0.2268928; Initial_Theta_reset = Initial_Theta_reset+0.0174533)
      {

        printf("Inital X, theta\n");

#endif

#ifndef SCAN

        Initial_x_reset = 0.4;//-1.0;//0.0;//1.7;
        Initial_Theta_reset = -0.0872665;//-0.1745329;//0.0349066;//-0.1570796;//-9 degrees

        printf("Inital X, theta\n");
        // -0.0174533; // -1 degrees
        // -0.0349066; // -2 degrees
        // -0.0523599; // -3 degrees
        // -0.0698132; // -4 degrees
        // -0.0872665; // -5 degrees
        // -0.1047198; // -6 degrees
        // -0.1221730; // -7 degrees
        // -0.1396263; // -8 degrees
        // -0.1570796; // -9 degrees
        // -0.1745329; // -10 degrees
        // -0.1919860; // -11 degrees
        // -0.2094395; // -12 degree

#endif


        double TimeElapsed = 0.0;
        float x,			/* cart position, meters */
        x_dot,			/* cart velocity */
        theta,			/* pole angle, radians */
        theta_dot;		/* pole angular velocity */
        vector  w,			/* vector of action weights */
        v,			/* vector of critic weights */
        e,			/* vector of action weight eligibilities */
        xbar;			/* vector of critic weight eligibilities */
        float p, oldp, rhat, r, yy;
        int box, i, y, steps = 0, failures=0, failed;


        //printf("Seed? ");
        // scanf("%d",&i);
        //srand(i);

        srand (time(NULL));

        /*--- Initialize action and heuristic critic weights and traces. ---*/
        for (i = 0; i < N_BOXES; i++)
          w[i] = v[i] = xbar[i] = e[i] = 0.0;



        /*--- Starting state is (0 0 0 0) ---*/
        x = x_dot = theta = theta_dot = 0.0;

        x = Initial_x_reset;
        theta = Initial_Theta_reset;


        /*--- Find box in state space containing start state ---*/
        box = get_box(x, x_dot, theta, theta_dot);



        ////////////initial weight
#ifndef RandomWeights
        weightActual_x_RL = 0.5;
        weightActual_theta_RL = 0.3;
        weightActual_x_dot_RL = 0.4;
        weightActual_theta_dot_RL = 0.1;
#endif

#ifdef RandomWeights
        weightActual_x_RL = ((HIGH_w_x-LOW_w_x)*((float)rand()/RAND_MAX))+LOW_w_x; // Input 0 (m) between 0 and 1.0
        weightActual_theta_RL = ((HIGH_w_x-LOW_w_x)*((float)rand()/RAND_MAX))+LOW_w_x; // Input 0 (m) between 0 and 1.0
        weightActual_x_dot_RL = ((HIGH_w_x-LOW_w_x)*((float)rand()/RAND_MAX))+LOW_w_x; // Input 0 (m) between 0 and 1.0
        weightActual_theta_dot_RL = ((HIGH_w_x-LOW_w_x)*((float)rand()/RAND_MAX))+LOW_w_x; // Input 0 (m) between 0 and 1.0
        printf("Inital Random weight, Wx %f Wtheta %f Wxdot %f Wthetadot %f \n",weightActual_x_RL,weightActual_theta_RL,weightActual_x_dot_RL,weightActual_theta_dot_RL);
#endif



        original_noise = 0.0;
        noise_p = 0.0;
        output_RL = 0.0;

        p = 0.0;
        rhat =0.0;
        oldp = 0.0;
        r = 0.0;

        e_x_RL_old =  0.0,
            e_theta_RL_old = 0.0,
            e_x_dot_RL_old = 0.0,
            e_theta_dot_old = 0.0;

        e_x_RL = 0.0,
            e_theta_RL = 0.0,
            e_x_dot_RL = 0.0,
            e_theta_dot_RL = 0.0;




        //TIME
        clock_t start = clock();


        /*--- Iterate through the action-learn loop. ---*/
        while (steps++ < MAX_STEPS && failures < MAX_FAILURES)
        {



          //Inputs: x, x_dot, theta, theta_dot;
          //x = m [-2.4,...,2.4]
          //x_dot = m [-2.4,...,2.4]
          //theta = radians [-0.209,...,0.209]
          //theta_dot = radians [-0.209,...,0.209]

          /******************** Convert Inputs to range [-1,...,+1]*****************************/

          //To scale to between +-1
          x_com = ((x-minX)/(maxX-minX))*2-1;

          if(x_com>1)
            x_com = 1;
          if(x_com<-1)
            x_com = -1;

          theta_com = ((theta-mintheta)/(maxtheta-mintheta))*2-1;

          if(theta_com>1)
            theta_com = 1;
          if(theta_com<-1)
            theta_com = -1;

          x_dot_com = ((x_dot-minX_dot)/(maxX_dot-minX_dot))*2-1;

          if(x_dot_com>1)
            x_dot_com = 1;
          if(x_dot_com<-1)
            x_dot_com = -1;

          Theta_dot_com =  ((theta_dot-mintheta_dot)/(maxtheta_dot-mintheta_dot))*2-1;

          if(Theta_dot_com>1)
            Theta_dot_com = 1;
          if(Theta_dot_com<-1)
            Theta_dot_com = -1;


          //All converted inputs to the network, e.g., ICO and RL actor and critic

          // x_com  = [-1,...,+1]
          // theta_com = [-1,...,+1]
          // x_dot_com = [-1,...,+1]
          // Theta_dot_com = [-1,...,+1]

          /*********************************************************************************/




          //--------------------------------------ACTOR--------------------------------------------------//
          // Call noise function
          //#### (1)
          original_noise = gauss(1.0); //[-1, 0, 1]
          if(original_noise >1)
            original_noise = 1;
          if(original_noise<-1)
            original_noise = -1;

          if (failed)
          {
            steps_noise = 1.0;
          }

          steps_noise++;


          //printf("p %f r %f rhat %f %d\n", p, r, rhat, steps);


          // USED NOISE from original noise * prediction value of critic (p)
          noise_p = original_noise*0.1;//steps;//*fabs(p);



          //#### (2)
          /////// OUTPUT of actor = linear combination from 4 inputs + noise*p//////////////////////
          output_RL = weightActual_x_RL*x_com +
              weightActual_theta_RL*theta_com +
              weightActual_x_dot_RL*x_dot_com +
              weightActual_theta_dot_RL*Theta_dot_com+noise_p;

          //outputShaped_Com = 2/(1+exp(-2*output_RL))-1.0; // NOT IMPORTANT


          //#### (3)
          //Set output of the RL actor to the output to drive a cart
          yy =  output_RL;



          //yy = outputShaped_Com;
          /*--- Choose action randomly, biased by current weight. ---*/
          //   y = (random < prob_push_right(w[box]));
          // fprintf(finputs_outputs, "%d" " " "%f" " " "%f" " " "%f" " " "%f" " " "%f" " " "%f" " " "%f" " " "%f"  " " "%f" " " "%f" " " "%d""\n", failures, weightActual_x_RL, weightActual_theta_RL, weightActual_x_dot_RL, weightActual_theta_dot_RL, x, theta, output_RL,noise_p,p,original_noise,steps);





          /*--- Update traces. ---*/



          //--- Remember e of current state ---//
          e_x_RL_old = e_x_RL;
          e_theta_RL_old = e_theta_RL;
          e_x_dot_RL_old = e_x_dot_RL;
          e_theta_dot_old = e_theta_dot_RL;

          e_x_RL = LAMBDAw_con * e_x_RL_old + (1.0 - LAMBDAw_con) *  yy  *x_com;
          e_theta_RL = LAMBDAw_con * e_theta_RL_old + (1.0 - LAMBDAw_con) * yy  *theta_com;
          e_x_dot_RL  = LAMBDAw_con * e_x_dot_RL_old + (1.0 - LAMBDAw_con) *  yy *x_dot_com;
          e_theta_dot_RL = LAMBDAw_con * e_theta_dot_old + (1.0 - LAMBDAw_con) * yy *Theta_dot_com;


          xbar[box] += (1.0 - LAMBDAv); // LAMBDAv = 0.8


          //Using no trace ONLY input and output correlation
          //e_x_RL = x_com*yy;
          //e_theta_RL = theta_com*yy;
          //e_x_dot_RL  = x_dot_com*yy;
          //e_theta_dot_RL = Theta_dot_com*yy;



          //e[box] += (1.0 - LAMBDAw) * (y - 0.5); // LAMBDAw	= 0.9 //NOT USE




          /*--- Remember prediction of failure for current state ---*/
          oldp = v[box];


          /*--- Apply action to the simulated cart-pole ---*/
          cart_pole(&yy, &x, &x_dot, &theta, &theta_dot);

          /*--- Get box of state space containing the resulting state. ---*/
          box = get_box(x, x_dot, theta, theta_dot);

          if (box < 0)
          {
            /*--- Failure occurred. ---*/
            failed = 1;
            failures++;
            //printf("Trial %d was %d steps. %f force\n", failures, steps, yy);


            steps = 0;

            /*--- Reset state to (0 0 0 0).  Find the box. ---*/
            // x = x_dot = theta = theta_dot = 0.0;

            x =  Initial_x_reset;
            theta = Initial_Theta_reset;

            x_dot = 0.0;
            theta_dot = 0.0;


            box = get_box(x, x_dot, theta, theta_dot);

            /*--- Reinforcement upon failure is -1. Prediction of failure is 0. ---*/
            r = -1.0;
            p = 0.0;
          }
          else
          {
            /*--- Not a failure. ---*/
            failed = 0;

            /*--- Reinforcement is 0. Prediction of failure given by v weight. ---*/
            r = 0;
            p= v[box];
          }




          /*--- Heuristic reinforcement is:   current reinforcement
	      + gamma * new failure prediction - previous failure prediction ---*/
          rhat = r + GAMMA * p - oldp; //GAMMA = 0.95



          for (i = 0; i < N_BOXES; i++)
          {
            /*--- Update all weights. ---*/
            // w[i] += ALPHA * rhat * e[i]; //NOT USE
            v[i] += BETA * rhat * xbar[i]; //BETA	= 0.1
            if (v[i] < -1.0)
              v[i] = v[i];

            if (failed)
            {
              /*--- If failure, zero all traces. ---*/
              //   e[i] = 0.; //NOT USE
              xbar[i] = 0.;
            }
            else
            {
              /*--- Otherwise, update (decay) the traces. ---*/
              //  e[i] *= LAMBDAw; //NOT USE
              xbar[i] *= LAMBDAv;
            }
          }



          //ALPHA		0.1//1000   /* Learning rate for action weights, w. */

          weightActual_x_RL += ALPHA * rhat * e_x_RL;
          weightActual_theta_RL +=  ALPHA * rhat * e_theta_RL;
          weightActual_x_dot_RL += ALPHA * rhat * e_x_dot_RL;
          weightActual_theta_dot_RL += ALPHA * rhat * e_theta_dot_RL;

          /* if (failed)
	{

	e_x_RL = 0.0;
	e_theta_RL = 0.0;
	e_x_dot_RL = 0.0;
	e_theta_dot_RL = 0.0;

	}*/

        }




        if (failures == MAX_FAILURES)

        {
          TimeElapsed = 100.0;
          printf("\n");
          printf("RL discrete Critic Pole not balance %d failures. x %f theta %f Bigloop %d\n",failures, Initial_x_reset, Initial_Theta_reset, Bigloop); //* 180/pi
          printf("Time elapsed: %f\n", TimeElapsed); // Time function

        }

        else
        {
          TimeElapsed = ((double)clock() - start) / CLOCKS_PER_SEC;

          printf("\n");
          printf("RL discrete Critic Pole balanced successfully for at least %d steps x %f theta %f Bigloop %d\n", steps, Initial_x_reset, Initial_Theta_reset,Bigloop);
          printf("Time elapsed: %f\n", TimeElapsed); // Time function

        }

#ifdef SCAN
        //if(TimeElapsed >100.0)
        //TimeElapsed  =100.0;




        if(Bigloop == 0)
        {

          fprintf(finputs_outputs, "\t %f",TimeElapsed);

          fprintf(fWeightX, "\t %f",weightActual_x_RL);
          fprintf(fWeightTheta, "\t %f",weightActual_theta_RL);

          fprintf(fWeightXdot, "\t %f",weightActual_x_dot_RL);
          fprintf(fWeightThetadot, "\t %f",weightActual_theta_dot_RL);



        }

        if(Bigloop == 1)
        {

          fprintf(finputs_outputs2, "\t %f",TimeElapsed);

          fprintf(fWeightX2, "\t %f",weightActual_x_RL);
          fprintf(fWeightTheta2, "\t %f",weightActual_theta_RL);

          fprintf(fWeightXdot2, "\t %f",weightActual_x_dot_RL);
          fprintf(fWeightThetadot2, "\t %f",weightActual_theta_dot_RL);



        }

        if(Bigloop == 2)
        {

          fprintf(finputs_outputs3, "\t %f",TimeElapsed);

          fprintf(fWeightX3, "\t %f",weightActual_x_RL);
          fprintf(fWeightTheta3, "\t %f",weightActual_theta_RL);

          fprintf(fWeightXdot3, "\t %f",weightActual_x_dot_RL);
          fprintf(fWeightThetadot3, "\t %f",weightActual_theta_dot_RL);



        }

        if(Bigloop == 3)
        {

          fprintf(finputs_outputs4, "\t %f",TimeElapsed);

          fprintf(fWeightX4, "\t %f",weightActual_x_RL);
          fprintf(fWeightTheta4, "\t %f",weightActual_theta_RL);

          fprintf(fWeightXdot4, "\t %f",weightActual_x_dot_RL);
          fprintf(fWeightThetadot4, "\t %f",weightActual_theta_dot_RL);



        }

        if(Bigloop == 4)
        {

          fprintf(finputs_outputs5, "\t %f",TimeElapsed);

          fprintf(fWeightX5, "\t %f",weightActual_x_RL);
          fprintf(fWeightTheta5, "\t %f",weightActual_theta_RL);

          fprintf(fWeightXdot5, "\t %f",weightActual_x_dot_RL);
          fprintf(fWeightThetadot5, "\t %f",weightActual_theta_dot_RL);



        }

        if(Bigloop == 5)
        {

          fprintf(finputs_outputs6, "\t %f",TimeElapsed);

          fprintf(fWeightX6, "\t %f",weightActual_x_RL);
          fprintf(fWeightTheta6, "\t %f",weightActual_theta_RL);

          fprintf(fWeightXdot6, "\t %f",weightActual_x_dot_RL);
          fprintf(fWeightThetadot6, "\t %f",weightActual_theta_dot_RL);



        }





      } //Extra loop Theta loop





      if(Bigloop == 0)
      {
        fprintf(finputs_outputs,"\n");

        fprintf(fWeightX,"\n");
        fprintf(fWeightTheta,"\n");

        fprintf(fWeightXdot,"\n");
        fprintf(fWeightThetadot,"\n");
      }

      if(Bigloop == 1)
      {
        fprintf(finputs_outputs2,"\n");

        fprintf(fWeightX2,"\n");
        fprintf(fWeightTheta2,"\n");

        fprintf(fWeightXdot2,"\n");
        fprintf(fWeightThetadot2,"\n");
      }

      if(Bigloop == 2)
      {
        fprintf(finputs_outputs3,"\n");

        fprintf(fWeightX3,"\n");
        fprintf(fWeightTheta3,"\n");

        fprintf(fWeightXdot3,"\n");
        fprintf(fWeightThetadot3,"\n");
      }

      if(Bigloop == 3)
      {
        fprintf(finputs_outputs4,"\n");

        fprintf(fWeightX4,"\n");
        fprintf(fWeightTheta4,"\n");

        fprintf(fWeightXdot4,"\n");
        fprintf(fWeightThetadot4,"\n");
      }

      if(Bigloop == 4)
      {
        fprintf(finputs_outputs5,"\n");

        fprintf(fWeightX5,"\n");
        fprintf(fWeightTheta5,"\n");

        fprintf(fWeightXdot5,"\n");
        fprintf(fWeightThetadot5,"\n");
      }

      if(Bigloop == 5)
      {
        fprintf(finputs_outputs6,"\n");

        fprintf(fWeightX6,"\n");
        fprintf(fWeightTheta6,"\n");

        fprintf(fWeightXdot6,"\n");
        fprintf(fWeightThetadot6,"\n");
      }



    } // Extra loop X distance loop







#endif

  }//Bigloop

  /********SAVE DATA****************************/
  fclose(finputs_outputs);
  fclose(finputs_outputs2);
  fclose(finputs_outputs3);
  fclose(finputs_outputs4);
  fclose(finputs_outputs5);
  fclose(finputs_outputs6);

  fclose(fWeightX);
  fclose(fWeightTheta);
  fclose(fWeightXdot);
  fclose(fWeightThetadot);

  fclose(fWeightX2);
  fclose(fWeightTheta2);
  fclose(fWeightXdot2);
  fclose(fWeightThetadot2);

  fclose(fWeightX3);
  fclose(fWeightTheta3);
  fclose(fWeightXdot3);
  fclose(fWeightThetadot3);

  fclose(fWeightX4);
  fclose(fWeightTheta4);
  fclose(fWeightXdot4);
  fclose(fWeightThetadot4);

  fclose(fWeightX5);
  fclose(fWeightTheta5);
  fclose(fWeightXdot5);
  fclose(fWeightThetadot5);

  fclose(fWeightX6);
  fclose(fWeightTheta6);
  fclose(fWeightXdot6);
  fclose(fWeightThetadot6);

  /********************************************/
}


/*----------------------------------------------------------------------
   cart_pole:  Takes an action (0 or 1) and the current values of the
 four state variables and updates their values by estimating the state
 TAU seconds later.
----------------------------------------------------------------------*/

/*** Parameters for simulation ***/

#define GRAVITY 9.8
#define MASSCART 1.0
#define MASSPOLE 0.1
#define TOTAL_MASS (MASSPOLE + MASSCART)
#define LENGTH 0.5		  /* actually half the pole's length */
#define POLEMASS_LENGTH (MASSPOLE * LENGTH)
#define FORCE_MAG 10.0
#define TAU 0.01//0.02	//Evolved = 0.01!!  /* seconds between state updates */ 
#define FOURTHIRDS 1.3333333333333


//cart_pole(action, x, x_dot, theta, theta_dot)
//int action;
//float *x, *x_dot, *theta, *theta_dot;


//cart_pole(yy, x, x_dot, theta, theta_dot, x_dot_dot, theta_dot_dot)
//float *x, *x_dot, *theta, *theta_dot, *yy, *x_dot_dot, *theta_dot_dot;
cart_pole(yy, x, x_dot, theta, theta_dot)
float *yy, *x, *x_dot, *theta, *theta_dot;
{
  //  float xacc,thetaacc,force,costheta,sintheta,temp;
  float xacc,thetaacc,force,costheta,sintheta,temp, thetaacc0, thetaacc1,xacc0, xacc1;

  // force = (action>0)? FORCE_MAG : -FORCE_MAG;

  force = *yy * FORCE_MAG;

  costheta = cos(*theta);
  sintheta = sin(*theta);

  temp = (force + POLEMASS_LENGTH * *theta_dot * *theta_dot * sintheta)
		             / TOTAL_MASS;




  //Original
  /*  thetaacc = (GRAVITY * sintheta - costheta* temp)
	       / (LENGTH * (FOURTHIRDS - MASSPOLE * costheta * costheta
                                              / TOTAL_MASS));
     xacc  = temp - POLEMASS_LENGTH * thetaacc* costheta / TOTAL_MASS;

   */


  //To reduce numerical error
  thetaacc0 = (GRAVITY * sintheta - costheta* temp)
	           / (LENGTH * (FOURTHIRDS - MASSPOLE * costheta * costheta
	               / TOTAL_MASS));

  //convert to reduce numerical error
  thetaacc1 = (int)(thetaacc0*100000);
  thetaacc = thetaacc1/100000;


  xacc0  = temp - POLEMASS_LENGTH * thetaacc* costheta / TOTAL_MASS;

  //convert to reduce numerical error
  xacc1 = (int)(xacc0*100000);
  xacc = xacc1/100000;

  //To reduce numerical error




  /*** Update the four state variables, using Euler's method. ***/

  *x  += TAU * *x_dot;
  *x_dot += TAU * xacc;
  *theta += TAU * *theta_dot;
  *theta_dot += TAU * thetaacc;
}

/*----------------------------------------------------------------------
   get_box:  Given the current state, returns a number from 1 to 162
  designating the region of the state space encompassing the current state.
  Returns a value of -1 if a failure state is encountered.
----------------------------------------------------------------------*/

#define one_degree 0.0174532	/* 2pi/360 */
#define six_degrees 0.1047192
#define twelve_degrees 0.2094384
#define fifty_degrees 0.87266

get_box(x,x_dot,theta,theta_dot)
float x,x_dot,theta,theta_dot;
{
  int box=0;

  if (x < -2.4 ||
      x > 2.4  ||
      theta < -twelve_degrees ||
      theta > twelve_degrees)          return(-1); /* to signal failure */

  if (x < -0.8)  		       box = 0;
  else if (x < 0.8)     	       box = 1;
  else		    	               box = 2;

  if (x_dot < -0.5) 		       ;
  else if (x_dot < 0.5)                box += 3;
  else 			               box += 6;

  if (theta < -six_degrees) 	       ;
  else if (theta < -one_degree)        box += 9;
  else if (theta < 0) 		       box += 18;
  else if (theta < one_degree) 	       box += 27;
  else if (theta < six_degrees)        box += 36;
  else	    			       box += 45;

  if (theta_dot < -fifty_degrees) 	;
  else if (theta_dot < fifty_degrees)  box += 54;
  else                                 box += 108;

  return(box);
}


/*----------------------------------------------------------------------
   gauss: Generate noise for exploration
----------------------------------------------------------------------*/


gauss(iii)
float iii;
{
  float sum;
  int i;


  // srand (time(NULL));

  /*
//very low exploration (low noise)!!!
for(sum = i = 0; i<4; i++) 
sum += (1.0*(double)rand()/(RAND_MAX));

return (sum-2.0);
   */



  //very low exploration (low noise)!!!
  for(sum = i = 0; i<6; i++)
    sum += (1.0*(double)rand()/(RAND_MAX));

  return (sum-3.0);


  /*
//low exploration (low noise)!!!
for(sum = i = 0; i<12; i++) 
sum += (1.0*(float)rand()/(RAND_MAX));

return (sum-6.0);
   */

  /* VERY HIGH exploration (high noise)!!!
for(sum = i = 0; i<120; i++) 
sum += (1.0*(double)rand()/(RAND_MAX));

return (sum-60.0);
   */

}







/*----------------------------------------------------------------------
  Result of:  cc -o pole pole.c -lm          (assuming this file is pole.c)
              pole
----------------------------------------------------------------------*/
/*  
Trial 1 was 21 steps.
Trial 2 was 12 steps.
Trial 3 was 28 steps.
Trial 4 was 44 steps.
Trial 5 was 15 steps.
Trial 6 was 9 steps.
Trial 7 was 10 steps.
Trial 8 was 16 steps.
Trial 9 was 59 steps.
Trial 10 was 25 steps.
Trial 11 was 86 steps.
Trial 12 was 118 steps.
Trial 13 was 218 steps.
Trial 14 was 290 steps.
Trial 15 was 19 steps.
Trial 16 was 180 steps.
Trial 17 was 109 steps.
Trial 18 was 38 steps.
Trial 19 was 13 steps.
Trial 20 was 144 steps.
Trial 21 was 41 steps.
Trial 22 was 323 steps.
Trial 23 was 172 steps.
Trial 24 was 33 steps.
Trial 25 was 1166 steps.
Trial 26 was 905 steps.
Trial 27 was 874 steps.
Trial 28 was 758 steps.
Trial 29 was 758 steps.
Trial 30 was 756 steps.
Trial 31 was 165 steps.
Trial 32 was 176 steps.
Trial 33 was 216 steps.
Trial 34 was 176 steps.
Trial 35 was 185 steps.
Trial 36 was 368 steps.
Trial 37 was 274 steps.
Trial 38 was 323 steps.
Trial 39 was 244 steps.
Trial 40 was 352 steps.
Trial 41 was 366 steps.
Trial 42 was 622 steps.
Trial 43 was 236 steps.
Trial 44 was 241 steps.
Trial 45 was 245 steps.
Trial 46 was 250 steps.
Trial 47 was 346 steps.
Trial 48 was 384 steps.
Trial 49 was 961 steps.
Trial 50 was 526 steps.
Trial 51 was 500 steps.
Trial 52 was 321 steps.
Trial 53 was 455 steps.
Trial 54 was 646 steps.
Trial 55 was 1579 steps.
Trial 56 was 1131 steps.
Trial 57 was 1055 steps.
Trial 58 was 967 steps.
Trial 59 was 1061 steps.
Trial 60 was 1009 steps.
Trial 61 was 1050 steps.
Trial 62 was 4815 steps.
Trial 63 was 863 steps.
Trial 64 was 9748 steps.
Trial 65 was 14073 steps.
Trial 66 was 9697 steps.
Trial 67 was 16815 steps.
Trial 68 was 21896 steps.
Trial 69 was 11566 steps.
Trial 70 was 22968 steps.
Trial 71 was 17811 steps.
Trial 72 was 11580 steps.
Trial 73 was 16805 steps.
Trial 74 was 16825 steps.
Trial 75 was 16872 steps.
Trial 76 was 16827 steps.
Trial 77 was 9777 steps.
Trial 78 was 19185 steps.
Trial 79 was 98799 steps.
Pole balanced successfully for at least 100001 steps
 */

