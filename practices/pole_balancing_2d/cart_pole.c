/***********************************************/
/*                                             */
/*  cart_pole.c                                */
/*                                             */
/*  Jun Morimoto 2010.6.5                      */
/* Poramate Manoonpong 06.12.2013              */
/*                                             */
/*  Limited use for the collaboration study    */
/*  between Univ. of Gottingen and ATR-BRI     */
/*                                             */
/***********************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include "cart_pole.h"

#define N_RKBUFF 7



/*************************************************************
 *	Define variables and class
 *************************************************************/


//1)----------- Simulation cart_pole
void cart_pole(int n, double t, double *x, double *xdot); 

//2)-----------Runge-Kutta method for cart pole simulation
void rkstep( void (*vf)(),	
    int n, double *y,
    double *pt, double dt,
    double *work);

double cp_time = 0.0;
double dt = 0.01;
double rk_buffer[XDIM*N_RKBUFF /*4x7 = 28 arrays*/];
int nloop = 1000000;

int xdim = XDIM /*4 inputs*/;
double force=0.0;

//3)----------initial weight values ICO
double kico[XDIM] = {0.0,  0.0,  0.0,  0.0};
double output = 0.0;
int reset = 0;

//-----ICO learning---//
double x_com = 0.0, theta_com = 0.0, x_dot_com = 0.0, theta_dot_com = 0.0;
//-------------------//


//3)-----------Sensory input limit
double xt_limit = 2.4; // 2.4 m
double theta_limit = 12.0/180.*M_PI; /* -+0.2094 = 12 deg*/

double xmin[XDIM /*=4 inputs*/];
double xmax[XDIM /*=4 inputs*/];


//4)-----------Clip function!!
#ifndef clip
#define	clip(x/*input*/,l /*lower limit*/,u /*upper limit*/)( ((x)<(l)) ? (l) : ((x)>(u)) ? (u) : (x))
//if x < l , x = l and if x > u, x = u, else  x = x
#endif

#ifndef min
#define min(x/*e.g., 1*/, y) ((x <= y) ? x : y)
#endif

#ifndef max
#define max(x/*e.g., 0*/, y) ((x >= y) ? x : y)
#endif


#define ico


#define animation //comment this to run without animation
#define learning //comment this to test the learned weights


//for actor critic learning
#define stopexploration
#define stopafter 1
#define number_of_reset 2


//For students
//Task learning two sets of initial conditions
//1)  initialX = -1.0 (m), initialTH = 0.1396263, (8 deg)
//2)  initialX = 1.0 (m), initialTH = 0.1396263, (8 deg)

//Extra:
//Can we learn these initial conditions  initialX = 2.0 (m), initialTH = 0.1745329, (10 deg)?
//If yes how?, If not why?


// if not scan and random
static int random_state_set = 0; // 1 = random // 0 = defined
double initialX = -1.0;
double initialV = 0.0;
double initialTH = 0.1396263;
double initialOM = 0.0;

double learningRate_ico_value = 0.1;  //0.01;0.001
double learningRate_ico = 0.1;


//Non random position
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

/*************************************************************
 *	ICO network
 *************************************************************/
void init_icolearning()
{
  printf("activate ico learning \n\n");
  //Input limitation MAX MIN
  xmax[_X] = xt_limit; // 2.4 m
  xmin[_X] = -xt_limit; // -2.4 m

  xmax[_V] = 3.5;
  xmin[_V] =-3.5;

  xmax[TH] = theta_limit; // 12/180*PI = 12 deg =   0.2094 rad
  xmin[TH] =-theta_limit; // -12/180*PI = -12 deg = -0.2094 rad

  xmax[OM] = 2.0;
  xmin[OM] =-2.0;
}


int main(int argc, char **argv)
{

  // Initial state////
  xt[_X /*input 0*/] = 0.0;
  xt[_V /*input 1*/] = 0.0;
  xt[TH /*input 2*/] = 0.0;
  xt[OM /*input 3*/] = 0.0;


  printf("Initial state:  xt = %f,  x_dot = %f,  th = %f, th_dot = %f\n", xt[_X /*input 0*/], xt[_V /*input 1*/], xt[TH /*input 2*/], xt[OM /*input 3*/]);

  srand(time( NULL));

  /*CONTOL POLICY and ICO learning*/
  init_icolearning();

#ifdef animation

  printf("ANIMATION");
  /*Simulation animation NO CONTROLLER ACTIVATED!*/
  gl_init_anim(argc, argv);

  /* GLUT main loop  */
  glut_loop();//-------->> call   simulation_loop();--- CALL CONTROLLER

#endif

#ifndef animation
  printf("NO ANIMATION");
  simulation_loop();//------------------------------------------------------------- CALL CONTROLLER
#endif

  return 0;
}

/*************************************************************
 *	Gaussian random variable: just a sum of uniform distribution
 *	Average = 0, Variance = 1
 *************************************************************/

double gauss()
{
  double	sum;
  int 	i;

  for( sum = i = 0; i < 12; i++) sum += (1.0*(double)rand()/(RAND_MAX));
  return( sum - 6.0);
}

int check_limit(double *x)
{
  int flag=0;

  if(x[_X] < -xt_limit || x[_X]  > xt_limit)
  {
    flag = 1;
  }

  if(x[TH] < -theta_limit || x[TH] > theta_limit)
  {
    flag = 1;
  }

  return flag;

}


/*************************************************************
 *	RESET INITIAL STATE after fail!!!!
 *************************************************************/
double uniform_dist()
{
  double value;
  double max_t = 1;
  double min_t = -1;

  value = ((max_t-min_t)*((double)rand()/RAND_MAX))+min_t; //Range [-1,...,+1]

  return value;
}

void reset_state(double *x)
{
  int i;

  for(i=0;i<XDIM;i++)
    xt[i] = 0.0;

  xt[_X] = 0.5*xt_limit*uniform_dist(); // 0.5*2.4*(+1 or -1) = half of limit
  xt[TH] = 0.5*theta_limit*uniform_dist(); // 0.5*0.2094*(+1 or -1) = half of limit

}




/*************************************************************
 *	CALL learning and controller repeat until stop!!
 *************************************************************/

static double acum_reward = 0.0;
static double balance_time = 0.0;
/* Termination criterion. */
#define MAX_ITER  100000 // 100000 1000 [sec] //2000 // 20 [sec]
#define MAX_TRIAL 1000//100000 //500

static int failure_flag = 1; //<--- initialize
int ntrial = 0;

static int save = 1;

FILE *fdataico;

//Set initial state option
int steps = 0;
int total_trials;

void simulation_loop()
{

  static int iter = 0;
  int i;
  FILE *fp;
  int m;

  if(save)
  {
    fdataico = fopen("save_dataico.dat","w");
    save = 0;
  };


#ifndef animation
  while(steps++ < MAX_ITER && ntrial < MAX_TRIAL && reset < number_of_reset)
  {
#endif

    /************************AFTER RESET********************************************************/
    if(failure_flag || iter >= MAX_ITER/2)
    {
      if(iter >= MAX_ITER/2)
        reset++;
      else
        ntrial++;

      if(random_state_set)
      {
        reset_state(xt);

        initialX = xt[_X];
        initialV = xt[_V];
        initialTH = xt[TH];
        initialOM = xt[OM];
      }
      else
      {

        initialV = 0.0001*uniform_dist();
        initialOM = 0.0001*uniform_dist();

        xt[_X] = initialX;
        xt[_V] = initialV;
        xt[TH] = initialTH;
        xt[OM] = initialOM;

      }

      // RESET values
      failure_flag = 0;
      iter = 0;
      steps = 0;
      balance_time = 0.0;
    }


    /************************DURING LEARNING STATE************************************************/
    //if(ntrial < MAX_TRIAL)
    if(steps++ < MAX_ITER && ntrial < MAX_TRIAL && reset < number_of_reset)
    {

      /* Inputs of the system*/
      printf("x =  %f, v = %f, th = %f, om = %f\n\n", xt[_X], xt[_V], xt[TH], xt[OM]);

#ifdef ico
      //-------ICO learning--------------//
      output_icolearning(xt, utico);
      //--------------------------------//
      //ICO learning
      force = 10.*utico[0];
#endif

      /* Cart Pole Dynamics */
      rkstep(&cart_pole, xdim, xt, &cp_time, dt, rk_buffer);

      /* Check state limit */
      failure_flag = check_limit(xt);
      balance_time += dt;

      total_trials = ntrial+reset;
      //ICO
      fprintf(fdataico,"%d %f %f %f %f %f %f %f %f %f %d\n",total_trials, xt[_X /*input 0*/],xt[_V /*input 1*/], xt[TH /*input 2*/],xt[OM /*input 3*/],utico[0], kico[0], kico[1], kico[2], kico[3],ntrial);

      iter++;

#ifdef ico

      printf("----ICO learning----- \n\n");
      if(random_state_set)
      {
        printf("//RANDOM Initial state:: xt[_X] = %f, xt[_V] = %f, xt[TH] = %f, xt[OM] = %f\n", initialX, initialV,initialTH*180/M_PI,initialOM);
      }
      else
      {
        printf("//Defined Initial state:: xt[_X] = %f, xt[_V] = %f, xt[TH] = %f, xt[OM] = %f\n", initialX, initialV,initialTH*180/M_PI,initialOM);
      }

      printf("kico[_X] = %f; kico[_V] = %f; kico[TH] = %f; kico[OM] = %f;\n", kico[_X],kico[_V],kico[TH],kico[OM]);

      printf("steps %d :: iter %d: total trials %d :: reset %d\n", steps, iter, total_trials, reset);
#endif

    }
    else
    {

      ////---------Print success or fail--------------////
      if (ntrial == MAX_TRIAL)
      {
        //TimeElapsed = 100.0;
        printf("\n");
        printf("Pole not balance %d failures\n",ntrial); //* 180/pi

      }

      else
      {
        //TimeElapsed = ((double)clock() - start) / CLOCKS_PER_SEC;
        printf("\n");
        printf("Pole balanced successfully for at least %d steps \n", steps);
        printf("kico[_X] = %f; kico[_V] = %f; kico[TH] = %f; kico[OM] = %f;\n", kico[_X],kico[_V],kico[TH],kico[OM]);
      }

      ////---------Print success or fail--------------////
      exit(0);
    }


#ifndef animation
  }
#endif

}


/*************************************************************
 *	ICO learning
 *************************************************************/

void output_icolearning(double *x_ico, double *u_ico)
{

  double maxX = 2.4;
  double minX = -2.4;

  double maxtheta = 0.2094384;
  double mintheta = -0.2094384;

  double maxX_dot = 3.5;
  double minX_dot = -3.5;

  double maxtheta_dot = 2.0;
  double mintheta_dot = -2.0;

  //To scale to between +-1

  x_com = ((x_ico[_X]-minX)/(maxX-minX))*2-1;

  if(x_com>1)
    x_com = 1;
  if(x_com<-1)
    x_com = -1;

  theta_com = ((x_ico[TH]-mintheta)/(maxtheta-mintheta))*2-1;

  if(theta_com>1)
    theta_com = 1;
  if(theta_com<-1)
    theta_com = -1;

  x_dot_com = ((x_ico[_V]-minX_dot)/(maxX_dot-minX_dot))*2-1;

  if(x_dot_com>1)
    x_dot_com = 1;
  if(x_dot_com<-1)
    x_dot_com = -1;

  theta_dot_com =  ((x_ico[OM]-mintheta_dot)/(maxtheta_dot-mintheta_dot))*2-1;

  if(theta_dot_com>1)
    theta_dot_com = 1;
  if(theta_dot_com<-1)
    theta_dot_com = -1;


  //------------START-For students: Create your own controller and learning here------------//
  // Recommendation using a linear controller!

  //1) Inputs to your controller//
  //  x_com  = [+-1]
  //  theta_com = [+-1]
  //  x_dot_com = [+-1]
  //  theta_dot_com = [+-1]

  //2) Output of your controller is "u_ico[0]"

  //3) Learning parameters: "kico[_X], kico[_V], kico[TH], kico[OM]";


  u_ico[0] = 1.0; // Example of output

  //------------END-For students: Create your own controller and learning here------------//

}

/************SIMULATION**************************************************************************/
#define FORCE_MAX 10.0
void cart_pole(int n, double t, double *x, double *xdot)
{

  double theta, theta_dot;
  double costheta, sintheta;
  double temp;
  double thetaacc, xacc;

  double sensorynoise;

  //sensorynoise = 0.001*uniform_dist();

  sensorynoise = 0.0;//0.000001*uniform_dist();

  theta = x[TH];//+sensorynoise;
  theta_dot = x[OM];

  costheta = cos(theta);
  sintheta = sin(theta);

  temp = (force+ POLEMASS_LENGTH * theta_dot * theta_dot * sintheta)/ TOTAL_MASS;

  thetaacc = (GRAVITY * sintheta - costheta* temp)
    		                / (LENGTH * (FOURTHIRDS - MASSPOLE * costheta * costheta/ TOTAL_MASS));

  xacc  = temp - POLEMASS_LENGTH * thetaacc* costheta / TOTAL_MASS;

  xdot[TH] = x[OM];
  xdot[_X] = x[_V];
  xdot[OM] = thetaacc;
  xdot[_V] = xacc;
}

/****
 *	Runge-Kutta
 ****/

void rkstep( void (*vf)(),	/* vector field vf( n, t, y, ydot) */
    int n, double *y,
    double *pt, double dt,
    double *work)	/* work area >= 7*n */
{
  double	*y2, *y3, *y4, *k1, *k2, *k3, *k4;
  double	dt2 = dt/2;
  int 	i;

  y2 = work;
  y3 = work + n;
  y4 = work + 2*n;
  k1 = work + 3*n;
  k2 = work + 4*n;
  k3 = work + 5*n;
  k4 = work + 6*n;

  vf( n, *pt, y, k1);
  for( i = 0; i < n; i++){
    y2[ i] = y[ i] + k1[ i]*dt2;
  }
  *pt += dt2;
  vf( n, *pt, y2, k2);
  for( i = 0; i < n; i++){
    y3[ i] = y[ i] + k2[ i]*dt2;
  }
  vf( n, *pt, y3, k3);
  for( i = 0; i < n; i++){
    y4[ i] = y[ i] + k3[ i]*dt;
  }
  *pt += dt2;
  vf( n, *pt, y4, k4);
  for( i = 0; i < n; i++){
    y[ i] += ( k1[ i] + 2*k2[ i] + 2*k3[ i] + k4[ i])*dt/6;
  }
}
