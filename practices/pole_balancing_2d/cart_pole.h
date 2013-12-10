/********************************/
/*                              */
/*   cart-pole.h          */
/*                              */
/*   2010.05.17                 */
/*  made by Jun Morimoto  */
/*                              */
/********************************/

/*coordinate*/
#define _X 0
#define _V 1
#define TH 2
#define OM 3

#define XDIM 4
#define UDIM 1

double xt[XDIM];
double ut[UDIM];
double utico[UDIM];


//----------------Simulation
#define GRAVITY 9.8
#define MASSCART 1.0
#define MASSPOLE 0.1
#define TOTAL_MASS (MASSPOLE + MASSCART)
#define LENGTH 0.5		  
#define POLEMASS_LENGTH (MASSPOLE * LENGTH)

#define FORCE_MAG 10.0
#define TAU 0.01
#define FOURTHIRDS 1.3333333333333

extern void simulation_loop();
void gl_init_anim(int argc, char **argv);
