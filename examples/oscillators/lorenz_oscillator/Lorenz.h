#ifndef __MATSUOKA_H__
#define __MATSUOKA_H__

// #################### Include header files #####################

#include <math.h>
#include <string.h>
#include <stdio.h>


///////// header files for save text to debug program/////////////

#include <iostream>
#include <fstream>
#include <string.h>


using namespace std;


class LorenzOscillator
{
  public:

    LorenzOscillator();
    ~LorenzOscillator();


    ofstream outFile;
    ofstream outFile2;
    ofstream outFile3;

    // --- Save text------------//
    ofstream saveFile1;
    ofstream saveFile2;
    ofstream saveFile3;
    ofstream saveFile4;
    ofstream saveFile5;
    ofstream saveFile6;
    //-------------------------//



};

#endif
