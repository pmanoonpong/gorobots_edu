//********************************************************
// hopfiledtest.cpp : main project file.
//
// Author: Sakyasingha Dasgupta 
//
//***********************************************************

//#include "stdafx.h"
#include <iostream>
using namespace std;

const int MAX_CLASSES = 2;    // Max. number of input classes
const int PATTERN_LENGTH = 8; // Max. number of 'bits' per pattern
const int MAX_TIME = 10;      // Max. 'time steps' for convergence


// This is the vocabulary of patterns which the inputs are compared to
const int X[MAX_CLASSES+1][PATTERN_LENGTH+1] =
          {{0,0,0,0,0,0,0,0,0},        // First row and column is dummy

           {0,1,1,1,1,-1,-1,-1,-1},    // Represents 11110000 --> Pattern 1
           {0,-1,-1,-1,-1,1,1,1,1}};   // Represents 00001111 --> Pattern 2


        // t[i][j] = Connection weight from node i to node j

int t[PATTERN_LENGTH+1][PATTERN_LENGTH+1];
int mu[MAX_TIME][PATTERN_LENGTH+1]; // Contains output of net at time 't'
                                    // First index really does run from 0

void assign_connection_weights ()
{ int sum;
  for (int i = 1; i <= PATTERN_LENGTH; i++)
  for (int j = 1; j <= PATTERN_LENGTH; j++)
   if (i == j)
    t[i][j] = 0;    // Weight=0 for connection from a node to itself
   else
    { sum = 0;
      for (int s = 1; s <= MAX_CLASSES; s++)
       sum += X[s][i] * X[s][j]; // Rule for weight adaptation

      t[i][j] = sum;
      std::cout<<t[i][j]<<" ";
    }

     std::cout<<"\n";
}

// Get the user to type a test pattern using 1 to represent 1 and 0 to
// represent -1. The pattern is read directly into the mu[0] slot of the
// time array
void get_test_pattern ()
{ char s[PATTERN_LENGTH+2]; // 1 extra because of array indices starting
                         // from 1 and 1 extra because of /0 character
                         // at end
  int valid;                 // Some compilers don't have a bool type
  int i;
  do
   {
                std::cout << "Please enter a pattern using 1 to represent 1 and  0 to "
          << "represent -1\n";
          std::cout << "You should enter " << PATTERN_LENGTH
          << " symbols and press Enter at the end.\n";
          std::cin >> s;
     // Go through the string to make sure each symbol is either 1 or 0

     valid = 1;       // Assume valid until proven otherwise
     // Translate string positions starting from 0 into mu
     // positions starting from 1
     for (i = 0; i < PATTERN_LENGTH; i++)
      switch (s[i])
       { case '1' : mu[0][i+1] = 1; break;
         case '0' : mu[0][i+1] = -1; break;
         default : valid = 0;
       }
   }
  while (valid == 0);
}

// Write the values of the mu array for time slot tt using * and .

void write_output (int tt)
{ for (int i = 1; i <= PATTERN_LENGTH; i++)
         if (mu[tt][i] == 1)
                 std::cout << "1";
         else
                 std::cout << "0";
   std::cout << "\n";
}

// Iterate the net, moving from one time slot to another and printing out
// the outputs at each stage

void iterate ()
{ int sum;
  for (int tt = 1; tt <= MAX_TIME; tt++)  // Go through all time slots
   { for (int j= 1; j <= PATTERN_LENGTH; j++)
      { sum = 0;
        for (int i = 1; i <= PATTERN_LENGTH; i++)
         sum += t[i][j] * mu[tt-1][i];
              // Now pass sum through hard-limiter so it is 1 or -1
        if (sum > 0)
         mu[tt][j] = 1;
        else
         mu[tt][j] = -1;
      }
     write_output(tt);
   }
}



int main ()
{ char k;
  assign_connection_weights();
  do
   { get_test_pattern();
     iterate();
          std::cout << "\n";
          std::cout << "Would you like another go? (Press Y or N, then Enter) : ";
std::cin >> k;
   }
  while (k != 'Y' && k != 'N');

  return 0;
}

