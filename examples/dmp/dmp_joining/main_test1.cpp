
#include <string.h>  
#include <stdio.h>   
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>

//#include "dmp.h"

#include <utils/dmp-framework/dmp.h>

using namespace std;

int main(){

   int dmp_dim=2;
   float err=0.01;

   int n_dmps=4;   
   

  
   float tau=1;
   float dt=0.005; //seconds
   int n=n_dmps*1; //number of kernels
   float sigma=0.05; //width of kernels


   float Tj[n_dmps]; //duration of segments
   Tj[0]=1.0;   
   Tj[1]=0.5;
   Tj[2]=1.5;
   Tj[3]=0.75;
   
   float T=0; for (int i=0; i<n_dmps; i++) {T=T+Tj[i];} //seconds - total time
   
   float sj_x[n_dmps]; //start points of segments
   float gj_x[n_dmps];  //end points of segments
   float sj_y[n_dmps]; 
   float gj_y[n_dmps];
   
   sj_x[0]=0;   sj_y[0]=0;
   gj_x[0]=10;  gj_y[0]=0;
   
   sj_x[1]=10;   sj_y[1]=0;
   gj_x[1]=10;  gj_y[1]=-5;
  
   sj_x[2]=10;   sj_y[2]=-5;
   gj_x[2]=-5;  gj_y[2]=-5;
   
   sj_x[3]=-5;   sj_y[3]=-5;
   gj_x[3]=-5;  gj_y[3]=2;

   
   FILE* fid;
   
   DMP *dmp = NULL;
   dmp = new DMP[dmp_dim];

   fid=fopen("dmp.dat","w+");
   int step=0;
   
  dmp[0].init_dmp(sj_x[0], gj_x[n_dmps-1], T, dt, tau, n, sigma);
  dmp[1].init_dmp(sj_y[0], gj_y[n_dmps-1], T, dt, tau, n, sigma);
  
   for (float t=0; t<=1.2*tau*T; t=t+dt){  
     
      dmp[0].calculate_one_step_dmp_joining(t, n_dmps, Tj, sj_x, gj_x);
      dmp[1].calculate_one_step_dmp_joining(t, n_dmps, Tj, sj_y, gj_y);
      
      float x=dmp[0].get_y();
      float y=dmp[1].get_y();
      
      float ed=sqrt((gj_x[n_dmps-1]-x)*(gj_x[n_dmps-1]-x)+(gj_y[n_dmps-1]-y)*(gj_y[n_dmps-1]-y));
      if (ed>err){
         fprintf(fid,"%f %f\n",x,y);
         printf("step=%i t=%f x=%f y=%f\n",step,t,x,y);
         step++; 
      }
      else{
         break;
      }
   }
 

   fclose(fid);
   
   delete [] dmp;
   dmp = NULL;

   return 0;
}

