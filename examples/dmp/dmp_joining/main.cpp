
#include <string.h>  
#include <stdio.h>   
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>

#include <utils/dmp-framework/dmp.h>

using namespace std;

int main(){

   int dmp_dim=2;
   float err=0.01;

   int n_dmps=4; //number of segments
       
   float tau=1;
   float dt=0.005; //seconds
   int nj[n_dmps]; // number of kernels for each segment
   nj[0]=20;
   nj[1]=20;
   nj[2]=20;
   nj[3]=20;
   int n=0; for (int i=0; i<n_dmps; i++) {n=n+nj[i];} //total number of kernels
   float sigma=0.05; //width of kernels

   float Tj[n_dmps]; //duration of segments
   Tj[0]=0.9450;   
   Tj[1]=0.7350;
   Tj[2]=0.6650;
   Tj[3]=0.7350;
   float T=0; for (int i=0; i<n_dmps; i++) {T=T+Tj[i];} //seconds - total time
   float sj_x[n_dmps]; //start points of segments
   float gj_x[n_dmps];  //end points of segments
   float sj_y[n_dmps]; 
   float gj_y[n_dmps];
   
   sj_x[0]=0.0280;   sj_y[0]=0.7120;
   gj_x[0]=1.2413;   gj_y[0]=0.3066;
   
   sj_x[1]=gj_x[0];  sj_y[1]=gj_y[0];    
   gj_x[1]=1.8307;   gj_y[1]=0.4499;      
   
   sj_x[2]=gj_x[1];  sj_y[2]=gj_y[1];    
   gj_x[2]=2.8183;   gj_y[2]=-0.0350;      

   sj_x[3]=gj_x[2];  sj_y[3]=gj_y[2];    
   gj_x[3]=3.4262;   gj_y[3]=0.1743;      
   
   
   FILE* fid;
   
   DMP *dmp = NULL;
   dmp = new DMP[dmp_dim];

   fid=fopen("dmp.dat","w+");
   int step=0;
   
   dmp[0].init_dmp(sj_x[0], gj_x[n_dmps-1], T, dt, tau, n, sigma);
   dmp[1].init_dmp(sj_y[0], gj_y[n_dmps-1], T, dt, tau, n, sigma);

   string tempStr; // loading weights
   float* w_x=NULL;
   float* w_y=NULL;
   w_x = new float[n];
   w_y = new float[n];
   int i=0;
   
   fstream fileIn("w_a.dat.txt");
   while (!fileIn.eof()){
     getline(fileIn,tempStr);
     sscanf(tempStr.c_str(),"%f %f", &w_x[i], &w_y[i]);
     i++;
   }
   fileIn.close();
   
   fstream fileIn2("w_b.dat.txt");
   while (!fileIn2.eof()){
     getline(fileIn2,tempStr);
     sscanf(tempStr.c_str(),"%f %f", &w_x[i], &w_y[i]);
     i++;
   }
   fileIn2.close();

   fstream fileIn3("w_c.dat.txt");
   while (!fileIn3.eof()){
     getline(fileIn3,tempStr);
     sscanf(tempStr.c_str(),"%f %f", &w_x[i], &w_y[i]);
     i++;
   }
   fileIn3.close();

   fstream fileIn4("w_b.dat.txt");
   while (!fileIn4.eof()){
     getline(fileIn4,tempStr);
     sscanf(tempStr.c_str(),"%f %f", &w_x[i], &w_y[i]);
     i++;
   }
   fileIn4.close();   
   
   for (i=0; i<n; i++){
     dmp[0].set_w(i, w_x[i]);
     dmp[1].set_w(i, w_y[i]);
   }
   
   printf("Weights\n");
   for (i=0; i<n; i++){
     printf("%12.4f %12.4f\n",dmp[0].get_w(i),dmp[1].get_w(i));
   }   
   
   
   delete [] w_x;
   w_x=NULL;
   delete [] w_y;
   w_y=NULL;
  
   dmp[0].scale_sigma(n_dmps, nj, Tj);  //scaling width of kernels
   dmp[1].scale_sigma(n_dmps, nj, Tj);
 
   printf("Sigmas\n");
   for (i=0; i<n; i++){
     printf("%12.4f %12.4f\n",dmp[0].get_sigma(i),dmp[1].get_sigma(i));
   } 
   
   dmp[0].scale_center(n_dmps, nj, Tj);  //scaling center of kernels
   dmp[1].scale_center(n_dmps, nj, Tj);
 
   printf("Centers\n");
   for (i=0; i<n; i++){
     printf("%12.4f %12.4f\n",dmp[0].get_c(i),dmp[1].get_c(i));
   }    
  
   for (float t=0; t<=1.2*tau*T; t=t+dt){  
      dmp[0].calculate_one_step_dmp_joining(t, n_dmps, Tj, sj_x, gj_x);
      dmp[1].calculate_one_step_dmp_joining(t, n_dmps, Tj, sj_y, gj_y);
      
      float x=dmp[0].get_y();
      float y=dmp[1].get_y();

      fprintf(fid,"%f %f\n",x,y);
      printf("step=%i t=%f x=%f y=%f\n",step,t,x,y);
      step++;      
      
      float ed=sqrt((gj_x[n_dmps-1]-x)*(gj_x[n_dmps-1]-x)+(gj_y[n_dmps-1]-y)*(gj_y[n_dmps-1]-y));
      if ((ed<err) & (t>=T)) {
	  break;
      }
   }
 

   fclose(fid);
   
   delete [] dmp;
   dmp = NULL;

   return 0;
}

