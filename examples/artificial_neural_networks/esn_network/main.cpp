/**
 * @author Sakya & Poramate 02.12.2013
 */


#include "esntest.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cmath>

//Set parameters (for Students)
#define NUMBER_DATA  4901 //number of total data (if using Learning mode 1-4 & Testing Data_4900.txt)
#define BPM_ITER   4901   //set it equal to the number of total data (if using Learning mode 1-4 & Testing Data_4900.txt)

//#define NUMBER_DATA  2001 //number of total data (if using Testing Data_2000.txt)
//#define BPM_ITER   2001   //set it equal to the number of total data (if using Testing Data_2000.txt)


//#define NUMBER_DATA  4001 //number of total data (Testing Data_4000.txt)
//#define BPM_ITER   4001   //set it equal to the number of total data (Testing Data_4000.txt)


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
float density;
float omega;
float phaseshift;
int steps;
int period;

float *input_vector;
float *train_vector;


//For students (example learning Sine^9 wave)
int dataset_type = 3; //1->Narma10, 2->Narma30, 3->sine wave to Sine^9 Wave, or 4->sine wave to Rectangle Wave
bool use_file_data = true; // if set this to true, esn will be trained using "TestingData_4900.txt or TestingData_2000.txt"


vector<double> m_r0_t;
vector<double> m_r1_t;

double sum(int len, float *array)
{  double sum = 0.0;
for (int i = 0 ; i < len ; i++)
{
  sum += array[i];
}

return sum;
}

// inputs is random data [0,0.5] and output 10 times step narma
void dataset_narma10(float *inputs, float *trainOutputs, int steps)
{
  srand(time(NULL));

  int order = 10;//0;

  for (int i =0;i< steps; i++)
  {
    inputs[i]= rand()/(RAND_MAX +1.0)*0.5;

    trainOutputs[i] = 0.1;
  }

  for (int n=order; n<steps; n++)
  {
    trainOutputs[n+1]= 0.3*trainOutputs[n] + 0.05*trainOutputs[n]*sum(9,trainOutputs) + 1.5*inputs[n-9]*inputs[n] + 0.1;
  }

}

// inputs is random data [0,0.5] and output 30 times step narma
void dataset_narma30(float *inputs, float *trainOutputs, int steps)
{
  srand(time(NULL));

  // Todo changes
  int order = 30;

  for (int i =0;i< steps; i++)
  {
    inputs[i]= rand()/(RAND_MAX +1.0)*0.5;

    trainOutputs[i] = 0.1;
  }

  for (int n=order; n<steps; n++)
  {
    trainOutputs[n+1]= 0.2*trainOutputs[n] + 0.04*trainOutputs[n]*sum(29,trainOutputs) + 1.5*inputs[n-29]*inputs[n] + 0.001;
  }

}

//generates sin wave input data
void setArrayToSineInput(float * array, int neurons, int steps, float phaseshift, float frequency)
{
  for (int i = 0; i< steps; i++)  //the array is set to a sine pattern
  {
    for (int j = 0;j<neurons;j++)
    {
      array[neurons*i+j] = sin(frequency*i+phaseshift*neurons*(0.5 - j));
    }
  }
}

//generates sine^9 output data
void setArrayToSineOutput(float * array, int neurons, int steps, float phaseshift, float frequency)
{
  for (int i = 0; i< steps; i++)  //the array is set to a sine pattern
  {
    for (int j = 0;j<neurons;j++)
    {
      array[neurons*i+j] = 0.5*pow(sin(frequency*i+phaseshift*neurons*(0.5 - j)),9); // change '9' to any other value to toggle the transformation
    }
  }
}

//generates a square wave output
void setArrayToRectangles(float * array, int neurons, int steps, int peakwidth, int period)
{
  for (int i = 0; i< steps; i++)  //the inputs are set to a sine pattern
  {
    for (int j = 0;j<neurons;j++)
    {
      if (i % period < peakwidth) array[i*neurons+j] = 1;
    }
  }
}



int main (int argc, char **argv)
{


  // ************ Essential parameter initialization **************
  omega = 0.05*3.141; //frequency
  phaseshift = 5*3.141;
  steps = BPM_ITER;
  period = 50;

  //*****************************************************************

  input_vector = new float [steps];
  train_vector = new float [steps];

  test = new TestESN();

  m_r0_t.resize(NUMBER_DATA);
  m_r1_t.resize(NUMBER_DATA);

  if (dataset_type == 1){

    dataset_narma10(input_vector, train_vector, steps);


  }

  else if (dataset_type == 2){

    dataset_narma10(input_vector, train_vector, steps);


  }

  else if (dataset_type == 3){

    setArrayToSineInput(input_vector, 1, steps, phaseshift, omega);
    setArrayToSineOutput(train_vector, 1, steps, 0, omega);


  }

  else if (dataset_type == 4){

    setArrayToSineInput(input_vector, 1, steps, phaseshift, omega);
    setArrayToRectangles(train_vector, 1, steps, 20, 80);


  }


  for (int i=1;i<BPM_ITER;i++) {

    if (use_file_data){
      //------------Read from the file--------------------------------------------//
      char str[10];

      //For students
      ifstream b_file ("TestingData_4900.txt"); // 2 column data, first column = input, second column = target
      //ifstream b_file ("TestingData_2000.txt"); // 2 column data, first column = input, second column = target
      //ifstream b_file ("TestingData_4000.txt"); // 2 column data, first column = input, second column = target

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

    }

    else {
      input = input_vector[i];
      target = train_vector[i];
    }

    //-----------Call ESN, recurrent network------------------//
    test->RecurrentNetwork(input,target);
    //--------------------------------------------------------//


    std::cout<<"counter"<<" "<<i<< "\n";

  }


  return 0;
}
