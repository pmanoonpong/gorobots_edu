/**
 * @author Sakya & Poramate 02.12.2013
 */


#include "esntest.h"
#include <string.h>
#include <stdio.h>
#include <vector>
#include <cmath>

//Set parameters (for Students)
#define NUMBER_DATA  2001 //number of total data
#define BPM_ITER	2001   //set it equal to the number of total data


///////////////////////////////////////////
TestESN* test;
int t;
int i_text_loop;
int ii;
bool initialized;
double input_temp;
double target_temp;
double input;
double target;
vector<double> m_r0_t;
vector<double> m_r1_t;


int main (int argc, char **argv)
{

  test = new TestESN();

  m_r0_t.resize(NUMBER_DATA);
  m_r1_t.resize(NUMBER_DATA);


  for (int i=1;i<BPM_ITER;i++) {


    //------------Read from the file--------------------------------------------//
    char str[10];
    ifstream b_file ("TestingData_2000.txt"); // 2 column data, first column = input, second column = target

    if(!initialized)
    {i_text_loop = 0;
    while(b_file>>str) //time first column
    {
      input_temp = atof(str);//input
      m_r0_t.at(i_text_loop) = input_temp;

      b_file>> str;
      target_temp = atof(str);//target
      m_r1_t.at(i_text_loop) = target_temp;

      i_text_loop++;
    }
    initialized = true;
    }

    if(ii<i_text_loop)
    {
      ii++;
    }
    else
    {
      ii=0;
    }

    input = m_r0_t.at(ii);
    target = m_r1_t.at(ii);
    //--------------------------------------------------------//



    //-----------Call ESN, recurrent network------------------//
    test->RecurrentNetwork(input,target);
    //--------------------------------------------------------//


    std::cout<<"counter"<<" "<<i<< "\n";

  }


  return 0;
}
