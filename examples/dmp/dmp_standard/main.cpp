
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

   //parameters for letter "a"
/*   float T=0.4; //seconds
   float tau=10;
   float dt=0.005; //seconds
   int n=15; //number of kernels for letter "a"
   float sigma=0.05; //width of kernels for letter "a"
   float w_sc=5.0;

   float xs=0.94; //cm
   float ys=0.76;
   float xe=1.34;
   float ye=0.09; */ 
   

   //parameters for tomas' signature
   float T=3.2; //seconds
   float tau=3;
   float dt=0.005; //seconds
   int n=100; //number of kernels for tomas' signature
   float sigma=0.01; //width of kernels for tomas' signature
   float w_sc=1.4;

   float xs=0.50; //cm
   float ys=2.50;
   float xe=10.00;
   float ye=0.10;
   

   FILE* fid;
   
   DMP *dmp = NULL;
   dmp = new DMP[dmp_dim];
   
   dmp[0].init_dmp(xs, xe, T, dt, tau, n, sigma);
   dmp[1].init_dmp(ys, ye, T, dt, tau, n, sigma);
  
//    dmp[0].set_weights_from_file("w_a_1.dat");
//    dmp[1].set_weights_from_file("w_a_2.dat");

   string tempStr;
//   fstream fileIn("w_a.dat");
   fstream fileIn("w_sign_tomas.dat");
   float* w_x=NULL;
   float* w_y=NULL;
   w_x = new float[n];
   w_y = new float[n];
   int i=0;
   while (!fileIn.eof()){
     getline(fileIn,tempStr);
     sscanf(tempStr.c_str(),"%f %f", &w_x[i], &w_y[i]);
     i++;
   }
   fileIn.close();
   
   for (i=0; i<n; i++){
     dmp[0].set_w(i, w_x[i]);
     dmp[1].set_w(i, w_y[i]);
   }
   delete [] w_x;
   w_x=NULL;
   delete [] w_y;
   w_y=NULL;
   

   fid=fopen("dmp.dat","w+");
   int step=0;
   for (float t=0; t<=1.0*tau*T; t=t+dt){  
      for (int i=0; i<dmp_dim; i++){
         dmp[i].calculate_one_step_dmp(t);
      }
      
      float x=dmp[0].get_y();
      float y=dmp[1].get_y();
      float ed=sqrt((xe-x)*(xe-x)+(ye-y)*(ye-y));
      if (ed>err){
         fprintf(fid,"%f %f\n",0.01*w_sc*(x-xs),0.01*w_sc*(y-ys));
         printf("step=%i t=%f x=%f y=%f\n",step,t,0.01*w_sc*(x-xs),0.01*w_sc*(y-ys));
         step++; 
      }
   }
   
   fclose(fid);
   
   delete [] dmp;
   dmp = NULL;

   return 0;
}

