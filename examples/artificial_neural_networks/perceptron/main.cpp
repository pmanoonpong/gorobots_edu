#include "percept.h"
#include "stdio.h"
#include "stdlib.h"

int main (int argc, char * argv[])
{

  float inputv1[]= {1.95,0.27,0.69,1.25};
  float wtv1[]= {2,3,3,2}, wtv2[]= {3,0,6,2};
  FILE * wfile, * infile;
  int num=0, vecnum=0, i;
  float threshold = 7.0;

//  if (argc < 2)
//  {
//    cerr << "Usage: percept Weightfile Inputfile";
//    exit(1);
//  }
  // open  files

//  wfile= fopen(argv[1], "r");
//  infile= fopen(argv[2], "r");


   wfile= fopen("weight.dat", "r");
   infile= fopen("input.dat", "r");


  if ((wfile == NULL) || (infile == NULL))
  {
    cout << " Can't open a file\n";
    exit(1);
  }


  cout<<"\nTHIS PROGRAM IS FOR A PERCEPTRON NETWORK WITH AN INPUT LAYER OF";
  cout<<"\n4 NEURONS, EACH CONNECTED TO THE OUTPUT NEURON.\n";
  cout<<"\nTHIS EXAMPLE TAKES REAL NUMBERS AS INPUT SIGNALS\n";

  //create the network by calling its constructor.
  //the constructor calls neuron constructor as many times as the number of
  //neurons in input layer of the network.

  cout<<"please enter the number of weights/vectors \n";
  cin >> vecnum;

  for (i=1;i<=vecnum;i++)
  {
    fscanf(wfile,"%f %f %f %f\n", &wtv1[0],&wtv1[1],&wtv1[2],&wtv1[3]);
    network h1(wtv1[0],wtv1[1],wtv1[2],wtv1[3]);
    fscanf(infile,"%f %f %f %f \n",
        &inputv1[0],&inputv1[1],&inputv1[2],&inputv1[3]);
    cout<<"this is vector # " << i << "\n";
    cout << "please enter a threshold value, eg 7.0\n";
    cin >> threshold;

    h1.onrn.actvtion(inputv1, h1.nrn);
    h1.onrn.outvalue(threshold);
    cout<<"\n\n";
  }

  fclose(wfile);
  fclose(infile);

  return 0;

}


