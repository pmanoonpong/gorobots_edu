/*
 * ngnet.cpp
 *
 *  Created on: Jun 15, 2011
 *      Author: poramate
 *      Updated by Bassel Zeidan DEC 10 2013
 */


#include <stdlib.h>
#include <math.h>
#include "ngnet.h"

//for save file and cout, cin
#include <iostream>
#include <string.h>

#if 0
typedef struct defunit{
	int ni,no;
	double cx[IN]; /* centers of basis*/
	double ac;
	double softac;
	double ivar[IN];
	double gx[IN];
	double w[OUT];  /* weight:w[no]*/
	double dw[OUT];
}Unit,*unit;

typedef struct defcell{
	int fstf;
	Unit cell[UNITNUM];
}Cell;
#endif


//for save file and cout, cin
using namespace std;


/*************************************************************
 *	Initialization RBF net
 *************************************************************/



//NGNet::NGNet(int inn, int outn){

//}



NGNet::NGNet(int ni, int no) {
	this->IN = ni;
	this->OUT = no;
}

int NGNet::init_incsbox(Cell *isb,int ni, int no)
{
	int i,j;
	Unit *curr;

	curr = isb->cell;

	curr->ni = ni;
	curr->no = no;
	std::cout<<"NUMBER OF OUTPUT ACOTOR = "<<curr->no<<"NUMBER OF INPUT ACOTOR = "<<curr->ni<<std::endl;
	return(1);
}

void NGNet::write_RBF_to_file(int Units_number, int IN, int out, Cell* C) {

}

int NGNet::reset_incsbox(Cell *isb)
{
	Unit *curr;
	int i,unum,nin,nout;

	isb->fstf = 1;

	curr = isb->cell;
	nin = curr->ni;
	nout = curr->no;

	for(i=0;i<nin;i++)
	{
		curr->cx[i]=0.0; // Center of RBF , e.g. cx[0], cx[1]
		curr->gx[i]=0.0; //
		curr->ivar[i] = 1.0; //Variance of each input, ivar[0], ivar[1]
	}

	for(i=0;i<nout;i++)
		curr->w[i]=0.0; //weight at output --> Vt

	curr->ac = 0.0; //neural activation of RBF
	curr->softac = 0.0; //neural activation of RBF with softmax

	for(i=0;i<nout;i++)
		curr->dw[i] = 0.0; //E_trace of neural activation of RBF with softmax

	return(1);
}

//Make box system with center and invarience
void NGNet::put_incsbox(Cell *isb,
		int ni /*number input = e.g., 2*/,
		int no /*number output = e.g., 1*/,
		double *xp /*input vector*/
		, double *Ivr /*wbasis vector = width of each input*/
		, int *nc /*neuron number*/)
{
	int i,j;
	Unit *curr;

	curr = isb->cell+(*nc);

	curr->ni = ni;
	curr->no = no;

	for(i=0;i<ni /*2 inputs*/;i++)
	{
		curr->cx[i] = xp[i]; // cx[0], cx[1]
		curr->gx[i] = 0.0;   // gx[0], gx[1]
		curr->ivar[i] = Ivr[i]; // Ivr[0], Ivr[1]
	}

	for(i=0;i<no /*1 output*/;i++)
		curr->w[i] = 0.0;

	(*nc)++; //nc= 3 x 3 = 9 neurons
   //nc = 100;
}


/*************************************************************
 *	Calculation Value function
 *************************************************************/

////CALL to calculate V --1)-->RETURN y[0] = Vt to find TDerror
void NGNet::incsbox_output(Cell *isb, double *x, double *y,int *nc)
{
	//double suma;
	Unit *curr;
	int i,unum;

	//-------1) Return activation of neurons (curr->softac) & weight (w[0])-----------//
	//--Reset the output of Critic net (1 output) to 0 before computing new value-----//

	incsbox_activate( isb, x, nc /*e.g., 3*3 = 9 neurons*/);



	curr = isb->cell;

	//-------2) Reset output to 0 ----------------------------------------------------//
	//--Reset the output of Critic net (1 output) to 0 before computing new value-----//

	for(i=0;i<curr->no /*no = 1*/;i++)
	{
		if(y != NULL /*not == 0 then reset to 0*/)
		{
			y[i]=0.0;
		}
	}

	//-------3) Calculate new output: "y[0] = w[0]*softac"---------------------------//
	//---BUT-if softac == 0 then y[0] = old_y[0] ------------------------------------//
	//--- w[0],  softac come from incsbox_activate() --------------------------------//



	for(unum=0;unum<*nc /*e.g., hidden neurons 3*3 = 9 neurons*/;unum++,curr++)
	{
		if(curr->softac == 0.0) continue;

		for(i=0;i<curr->no/*1 output*/;i++) {
			y[i]+= (curr->w[i])*(curr->softac);
			//std::cout<<"w = "<<curr->w[i]<<std::endl;
		}
			//y[0] = w[0]*softac;
	}


}

///--2) Cal neural activation with softmax operation-->RETURN [curr->softac]
double NGNet::incsbox_activate( Cell *isb, double *x, int *nc)
{
	int i,unum,nin;
	double suma=0.0;
	double suma_set = 1.0;
	double sumg[IN /*input, e.g., 2*/];
	Unit *curr;



	curr=isb->cell;
	nin = curr->ni/*number input = e.g., 2*/;

	//----1) reset first
	for(i=0;i<nin /*number input = e.g., 2*/;i++)
		sumg[i]=0.0; // sumg[0], [1] = 0.0


	//----2) cal sum of all hidden neural activation
	for (unum=0;unum< *nc /*e.g., 9 neurons*/;unum++,curr++)
	{
		incsbox_unit(curr,x); //return ac = neuron activation
		suma+= curr->ac; // sum all ac for softmax operation

		//////////////////////////////----------------WHAT sumg[0], sumg [1] for ???
		for(i=0;i<nin/*2*/;i++)
			sumg[i]+=curr->gx[i]*curr->ac;
	}

	//----3) cal find neural activation of each hidden neuron with softmax
	curr=isb->cell;
	for(unum=0;unum<*nc /*e.g., 9*/;unum++,curr++)
	{
		if(suma == 0.0)
		{
			//Prevent curr->softac to infinity ; set suma = 1, curr-softac = curr->ac/1
			curr->softac = (curr->ac)/suma_set;
			for(i=0;i<nin /*2*/;i++)
				curr->gx[i]=0.0;
		}
		else
		{
			//Softmax operation
			curr->softac = (curr->ac)/suma;

			//Softmax operation with threshold -->0.001
			double active_thresh = 0.01;//0.001; // Threshold of softmax
			if(curr->softac < active_thresh)//active_thresh)
			{
				curr->softac = 0.0;
			}

			//printf("curr->softac %f\n", curr->softac);

			for(i=0;i<nin;i++)
				curr->gx[i]=curr->gx[i]-sumg[i]/suma;
			//////////////////////////////----------------WHAT gx[0], gx[1] for ???

		}
	}
	/*return(suma);*/
}

///--3) Cal neural activation RBF-->RETURN ac = neural activation
void NGNet::incsbox_unit(Unit *cunit, double *x)
{
	double ac=0.0;
	double ui[IN];


	int i,j,k,nin;

	nin = cunit->ni; // 2 inputs


	//----1) cal input distance from the center of RBF
	for(i=0;i<nin;i++){
		ui[i] = x[i] - cunit->cx[i]; // "e.g., u[0] = input[0]-center[0]"
	}

	//  //#ifdef PEND
	//  //  ui[0] = circ(ui[0],-M_PI,M_PI);
	//  //#endif

	//----2) cal neual activation RBF e.g., of 9 hidden neurons --> ac will be nine different values!!
	for(i=0;i<nin;i++)
	{
		// "e.g., ac = (input[0]-center[0])^2*(1/sigma[0]^2)+(input[1]-center[1])^2*(1/sigma[1]^2)"
		ac += ui[i]*ui[i]*(cunit->ivar[i])*(cunit->ivar[i]);

		// "e.g., gx[0] = -(input[0]-center[0])*(1/sigma^2)"
		cunit->gx[i] = -ui[i]*(cunit->ivar[i])*(cunit->ivar[i]);

	}

	cunit->ac = (double)(exp(-ac/2.0)); //Final neuron activation!!

}



/*************************************************************
 *	Calculation parameters for changing actor and critic weights
 *************************************************************/

///--1) Cal value function trace = low pass filter of value function
void NGNet::incsbox_trace(Cell *isb, double *x, double lambda /*trace*/, int *nc)
{
	int k;
	int nout;
	int unum;
	double y[OUT];
	Unit *curr;

	incsbox_output(isb,x,y,nc); //???????????????????????????????HELP

	curr=isb->cell;

	nout = curr->no /*= 1 output of actor*/;

	for(unum=0;unum<*nc /*number of hidden neuron "adaptive initial = 9"*/;unum++,curr++)
	{
		for(k=0;k<nout /*number of output neuron*/;k++)
		{
			curr->dw[k] *= lambda;
			curr->dw[k] += (1-lambda)*(curr->softac);
		}
	}
}

/*************************************************************
 *	Learning mechanism of critic weights with adaptive GSBFN,
 *	Gaussian Softmax Basis Function, Morimoto
 * ref. Reinforcement learning of dynamic motor sequence:
 * Learning to stand up
 *************************************************************/

///--2) Cal Update Weights of critic -->Learning mechanism
double NGNet:: incsbox_update(
				Cell *isb,
				double *x,
				double *error/*-TDerror*/,
				double rate,
				int *nc,
				double *Ivr,
				double Thr /*e.g. 0.1*/ ,
				double near /*e.g. 0.6*/
				)

{
	double y[OUT],err[OUT],ui[IN];
	int nin,nout,k,i,j,unum,errf;
	double serr,ac,msqr,maxac=0.0,sume=0.0,ca;
	Unit *curr;
	double dJ;
	double dJ_dc;
	double ivar_up,cx_up,ivar_max,cx_max;

	curr = isb->cell;

	nin = curr->ni;  //number of inputs // 2
	nout = curr->no; //number of output // 1

	//-1)  Initialization of the first state only one time!!!
	if(isb->fstf)
	{
		for(i=0;i<nin;i++)
		{
			curr->cx[i]=x[i];
			curr->ivar[i] = Ivr[i];
		}

		for(i=0;i<nout;i++)
			curr->w[i]=0.0;
	}

	isb->fstf = 0;

	/******************Adaptive weights*********************************************************/

	//-2)  Cal Vt
	incsbox_output(isb,x,y,nc); // Return y[0] = Vt

	for(unum=0;unum<*nc;unum++,curr++)
	{

		for(k=0;k<nout /*1*/;k++){
			curr->w[k]-=rate*error[k]*(curr->dw[k]); /*update weight*/

		}
	}

	return( 0);
}

double NGNet:: incsbox_update_actor(
				Cell *isb,
				double *x,
				double *exp,
				double *error/*-TDerror*/,
				double rate,
				int *nc,
				double *Ivr,
				double Thr /*e.g. 0.1*/ ,
				double near /*e.g. 0.6*/
				)
{
	double y[OUT],err[OUT],ui[IN];
	int nin,nout,k,i,j,unum,errf;
	double serr,ac,msqr,maxac=0.0,sume=0.0,ca;
	Unit *curr;
	double dJ;
	double dJ_dc;
	double ivar_up,cx_up,ivar_max,cx_max;

	curr = isb->cell;

	nin = curr->ni;  //number of inputs // 2
	nout = curr->no; //number of output // 1

	//-1)  Initialization of the first state only one time!!!
	if(isb->fstf)
	{
		for(i=0;i<nin;i++)
		{
			curr->cx[i]=x[i];
			curr->ivar[i] = Ivr[i];
		}

		for(i=0;i<nout;i++)
			curr->w[i]=0.0;
	}

	isb->fstf = 0;

	/******************Adaptive weights*********************************************************/

	//-2)  Cal Vt
	incsbox_output(isb,x,y,nc); // Return y[0] = Vt

	//-4) Weight adaptation
	for(unum=0;unum<*nc;unum++,curr++)
	{

		for(k=0;k<nout /*1*/;k++){
			curr->w[k]-=rate*error[k]*exp[0]*(curr->dw[k]);

		}
	}
	return( 0);
}


double NGNet:: incsbox_update_v_action_pairs(
				Cell *isb,
				double *x,
				double exp,
				double error/*-TDerror*/,
				double rate_v,
				double rate_actor,
				int *nc,
				double *Ivr,
				double Thr /*e.g. 0.1*/ ,
				double near /*e.g. 0.6*/,
				bool update_actor
				) {
	double y[OUT],err[OUT],ui[IN];
	int nin,nout,k,i,j,unum,errf;
	double serr,ac,msqr,maxac=0.0,sume=0.0,ca;
	Unit *curr;
	double dJ;
	double dJ_dc;
	double ivar_up,cx_up,ivar_max,cx_max;

	curr = isb->cell;

	nin = curr->ni;  //number of inputs // 2
	nout = curr->no; //number of output // 1

	//-1)  Initialization of the first state only one time!!!
	if(isb->fstf)
	{
		for(i=0;i<nin;i++)
		{
			curr->cx[i]=x[i];
			curr->ivar[i] = Ivr[i];
		}

		for(i=0;i<nout;i++)
			curr->w[i]=0.0;
	}

	isb->fstf = 0;

	/******************Adaptive weights*********************************************************/

	//-2)  Cal Vt
	incsbox_output(isb,x,y,nc); // Return y[0] = Vt

	//-4) Weight adaptation
	for(unum=0;unum<*nc;unum++,curr++)
	{

		for(k=0;k<nout /*1*/;k++){
			if (k == 0)
				curr->w[k]+=rate_v*error*(curr->dw[k]);
			else if (update_actor) curr->w[k]+=rate_actor*exp*(curr->dw[k]);
		}
	}
	return( 0);
}
