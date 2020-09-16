//*******************************************
//*                                         *
//*   velocity regulating network (VRN)     *
//*                                         *
//*******************************************
// author: worasuchad haomachai
// contract: haomachai@gmail.com,
// data: 4/02/2020
// version: 1.0.0

#include "VRN.h"

VRN::VRN()
{

    // initial class attributes
    this->output1 = 0.0;
    this->input_x = 0.0;
    this->input_y = 0.0;
    this->w1358   = 0.0;
    this->w2467   = 0.0;
    this->w9_10   = 0.0;
    this->w11_12  = 0.0;
    this->bias    = 0.0;
    this->h1      = 0.0;
    this->h2      = 0.0;
    this->h3      = 0.0;
    this->h4      = 0.0;
}

// set parameter which define the properties of single VRN
// set the initial parameter of VRN
void VRN::setParameter(float output1,float w1358,float w2467,
    float w9_10,float w11_12,float bias){

    this->output1  = output1;
    this->w1358    = w1358;
    this->w2467    = w2467;
    this->w9_10    = w9_10;
    this->w11_12   = w11_12;
    this->bias     = bias;
}

// let VRN run for once
// function that updates the output of the VRN
void VRN::run(float input_x, float input_y){

    this->input_x = input_x;
    this->input_y = input_y;

    this->h1 = tanh(this->w1358*this->input_x + this->w1358*this->input_y + this->bias);
    this->h2 = tanh(this->w2467*this->input_x + this->w2467*this->input_y + this->bias);
    this->h3 = tanh(this->w1358*this->input_x + this->w2467*this->input_y + this->bias);
    this->h4 = tanh(this->w2467*this->input_x + this->w1358*this->input_y + this->bias);

    this->output1 = tanh(this->w9_10*this->h1  + this->w9_10*this->h2 +
                         this->w11_12*this->h3 + this->w11_12*this->h4);
}


// function that return the output signal of the VRN

float VRN::getSignal(){
  return this->output1;
}
