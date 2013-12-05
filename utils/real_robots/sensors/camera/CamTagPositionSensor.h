/*
 * CamTagPositionSensor.h
 *
 *  Created on: June 4, 2012
 *      Author: Ren Guanjiao
 */

#ifndef CAMTAGPOSITIONSENSOR_H_
#define CAMTAGPOSITIONSENSOR_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include "Client.h"
#include <iostream>

class CamPosiSensor {
  public:
    CamPosiSensor(int CamNum);
    ~CamPosiSensor();
    double* CaptureFrame();
    void ShowImage();
    bool CameraOpened() {
      return bResult;
    }

  private:
    bool bResult;
    Client* mylink;
    CvCapture* capture;
    IplImage* frame;
    IplImage* img;
    char cImgH[255];
    char cImgW[255];
    char cTagsNum[255];
    int tagsNum;
    double tagInfo_old[8];

};

#endif
