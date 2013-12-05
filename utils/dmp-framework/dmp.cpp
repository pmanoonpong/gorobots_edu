
#include <string.h> 
#include <stdio.h>   
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>

#include "dmp.h"

using namespace std;

DMP::DMP(){
   alpha_z = ALPHA_Z;
   beta_z = BETA_Z;
   alpha_v = ALPHA_V;
   alpha_z = ALPHA_Z;
   alpha_w = ALPHA_W;
   c = NULL;
   sigma = NULL;
   w = NULL;
}

DMP::~DMP(){
   delete [] c;
   c = NULL;
   delete [] sigma;
   sigma = NULL;
   delete [] w;
   w = NULL;
}

void DMP::set_weights_from_file(const char* fname){
   string tempStr;
   int i=0;
   fstream fileIn(fname);
   while (!fileIn.eof()){
     getline(fileIn,tempStr);
     sscanf(tempStr.c_str(),"%f", &w[i++]);
   }
   fileIn.close();
}

void DMP::init_dmp(float start, float goal, float total_t, float delta_t, float temp_scaling, int n_kernels, float width){
   s=start;
   g=goal;
   T=total_t;
   dt=delta_t; // Sampling frequency of your system
   tau=temp_scaling;
   n=n_kernels;
   v=1;
   r=s;
   f=0;
   y=s;
   z=0;

   c = new float[n];  
   sigma = new float[n];
   w = new float[n];
  
   for (int i=0; i<n; i++){
      c[i]=i/(float)(n-1);
      sigma[i]=width;
      w[i]=0;
   }
}

void DMP::scale_sigma(int n_dmps, int *nj, float *Tj){
  int nj_cum=0;
  int a=0;
  int b=0;
  
  for(int j=0; j<n_dmps; j++){
     nj_cum=nj_cum+nj[j];
     if (j==0){
        a=0;
	b=nj[0];
     }
     else{
       a=nj_cum-nj[j];
       b=nj_cum;
     }
     for(int i=a; i<b; i++){
        sigma[i]=sigma[i]*(Tj[j]/T);
     }
  }
}

void DMP::scale_center(int n_dmps, int *nj, float *Tj){
   int a=0;
   int b=0;
   int nj_cum=0;
   float Tj_cum=0;
   
   for(int j=0; j<n_dmps; j++){
      nj_cum=nj_cum+nj[j];
      if (j==0){
         a=0;
	 b=nj[0];
      }
      else{
       a=nj_cum-nj[j];
       b=nj_cum;
      }
      int k=0;
      for(int i=a; i<b; i++){
         c[i]=(Tj[j]*(k/(float)(nj[j]-1)) + Tj_cum)/T;
	 k++;
      }
      Tj_cum=Tj_cum+Tj[j];
   }
}

//----------Using only this for standard DMP-------------------------------//
void DMP::calculate_one_step_dmp(float t){
   float dv;
   float dr;
   float dy;
   float dz;
   float a;
   float psi;
   float sum_psi;
   
   if (t==0){
      v=1;
      r=s;
      y=s;
      z=0;
      f=0; 
   }
   else{

     //-----------Calculating v to kill the last end point smoothly at the end position using inverse sigmoid function------------//

     a=exp((alpha_v/dt)*(tau*T-t));
      dv=-(alpha_v*a)/((1+a)*(1+a)); //Inverse sigmoid
      if (isnan(dv)){ //A non-zero value (true) if x is a NaN value; and zero (false) otherwise.
         dv=0;
      }    
      v=v+dv;

      //-----------Calculating r (base line->goal)------------------//
      if (t<=tau*T){
        // Following trajectory in a linear line
         dr=(1/tau)*(dt/T)*(g-s); // (goal-start point)/total Time (T), tau = scaling for faster or slower updated step
      }
      else{
         dr=0;
      }
      r=r+dr;      
      //-----------Calculating r (base line->goal)------------------//
      
      //-----------Calculating f (trajectory)------------------//
      // w[i] = learned weight!!!! need to be generated for different trajectories
      f=0;
      sum_psi=0;
      for (int i=0; i<n; i++){
         a=t/(tau*T)-c[i];
         psi=exp(-a*a/(2*sigma[i]*sigma[i])); //RBF kernel
         sum_psi=sum_psi+psi;
         f=f+psi*w[i]*v;
      }
      f=f/sum_psi;
      //-----------Calculating f (trajectory)------------------//
      
      //-----------Calculating DMP ---------------------------//
      //Main Eq. (1) Velocity change (acc.)
      //alpha_z = 0.95, beta_z = alpha_z/2, tau=3
      //y & z are calculated

      dz=(1/tau)*(alpha_z*(beta_z*(r-y)-z)+f);
      z=z+dz;
      //Main Eq. (2) Position change (velocity)
      dy=(1/tau)*z; 
      y=y+dy;      
   }
}


//----------Using only this for Joining DMP-------------------------------//
void DMP::calculate_one_step_dmp_joining(float t, int n_dmps, float* Tj, float* sj, float* gj){
   float dv;
   float dr;
   float dy;
   float dz;
   float a;
   float psi;
   float sum_psi;
   float* Ts=NULL;
   
   Ts = new float[n_dmps];
   Ts[0]=Tj[0];
   for (int i=1; i<n_dmps; i++){
      Ts[i]=Ts[i-1]+Tj[i];
   }
   
   if (t==0){
      v=1;
      r=s;
      y=s;
      z=0;
      f=0; 
   }
   else{
     
      a=exp((alpha_v/dt)*(tau*T-t));
      dv=-(alpha_v*a)/((1+a)*(1+a));
      if (isnan(dv)){
         dv=0;
      }    
      v=v+dv;

      
      int j=0;
      for (int k=0; k<(n_dmps-1); k++){
         if ( (t>tau*Ts[k]) && (t<=tau*Ts[k+1]) ){
	    j=k+1;
	    break;
	 }
      }
      
      if (t<=tau*T){
         dr=(1/tau)*(dt/Tj[j])*(gj[j]-sj[j]);
      }
      else{
         dr=0;
      }
      r=r+dr;      
      
      f=0;
      sum_psi=0;
      for (int i=0; i<n; i++){
         a=t/(tau*T)-c[i];
         psi=exp(-a*a/(2*sigma[i]*sigma[i])); 
         sum_psi=sum_psi+psi;
         f=f+psi*w[i]*v;
      }
      f=f/sum_psi;      
      
      dz=(1/tau)*(alpha_z*(beta_z*(r-y)-z)+f);
      z=z+dz;
      
      dy=(1/tau)*z; 
      y=y+dy;      
   }
   delete [] Ts;
   Ts = NULL;
}
