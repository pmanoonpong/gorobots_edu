// Sine

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>


#include "Sine.h"



#define pi 3.14159265

int main(int argc, char* argv[])
{





  ofstream saveFile1;
  saveFile1.open("ReadSensors1.txt",ios::out);




  printf("initial\n");


  double sine[1001];
  double a;
  double x;


  do{




    for(int i=0;i<=1000;i++){


      a = sin(pi/180 * i);



      //x = 1 * sin(2*pi/100*100*i);




      printf("CPG %f \n",a);//, Theta_dot, Theta);

      saveFile1 <<a<< "   \n" << flush; //SAVE DATA
    }


  }while(1==1);


  return 0;
}





