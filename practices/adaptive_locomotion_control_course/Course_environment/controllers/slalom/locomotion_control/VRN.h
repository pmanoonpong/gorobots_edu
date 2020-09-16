//*******************************************
//*                                         *
//*   velocity regulating network (VRN)     *
//*                                         *
//*******************************************
// author: worasuchad haomachai
// contract: haomachai@gmail.com,
// data: 4/02/2020
// version: 1.0.0

#ifndef VRN_H
#define VRN_H

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <stdint.h>

using namespace std;


class VRN {
public:

    VRN();

    // set the initial parameter of VRN
    void setParameter(float output1,float w1358,float w2467,
        float w9_10,float w11_12,float bias);

    // function that updates the output of the VRN
    void run(float input_x, float input_y);

    /* function that return the output signal of the VRN
    network */
    float getSignal();

private:
    // private attribute
    // weigth and activity
    float output1;
    float input_x;
    float input_y;
    float w1358;
    float w2467;
    float w9_10;
    float w11_12;
    float bias;
    float h1;
    float h2;
    float h3;
    float h4;
};


#endif //VRN_H
