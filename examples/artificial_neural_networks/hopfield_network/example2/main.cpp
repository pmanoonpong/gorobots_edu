
#include "hop.h"

int main ()
{
int patrn1[]= {1,0,1,0},i;
int wt1[]= {0,-3,3,-3};
int wt2[]= {-3,0,-3,3};
int wt3[]= {3,-3,0,-3};
int wt4[]= {-3,3,-3,0};

cout<<"\nTHIS PROGRAM IS FOR A HOPFIELD NETWORK WITH A SINGLE LAYER OF";
cout<<"\n4 FULLY INTERCONNECTED NEURONS. THE NETWORK SHOULD RECALL THE";
cout<<"\nPATTERNS 1010 AND 0101 CORRECTLY.\n";

//create the network by calling its constructor.
// the constructor calls neuron constructor as many times as the number of
// neurons in the network.
network h1(wt1,wt2,wt3,wt4);

//present a pattern to the network and get the activations of the neurons
h1.activation(patrn1);

//check if the pattern given is correctly recalled and give message
for(i=0;i<4;i++)
  {
  if (h1.output[i] == patrn1[i])
    cout<<"\n pattern= "<<patrn1[i]<<
    "  output = "<<h1.output[i]<<"  component matches";
  else
    cout<<"\n pattern= "<<patrn1[i]<<
    "  output = "<<h1.output[i]<<
    "  discrepancy occured";
  }
cout<<"\n\n";
int patrn2[]= {0,1,0,1};
h1.activation(patrn2);
for(i=0;i<4;i++)
  {
  if (h1.output[i] == patrn2[i])
    cout<<"\n pattern= "<<patrn2[i]<<
    "  output = "<<h1.output[i]<<"  component matches";
  else
    cout<<"\n pattern= "<<patrn2[i]<<
    "  output = "<<h1.output[i]<<
    "  discrepancy occurred";
       }
return 0;
}
