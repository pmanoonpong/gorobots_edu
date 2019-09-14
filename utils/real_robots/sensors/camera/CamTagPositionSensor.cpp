/*
 * CamTagPositionSensor.cpp
 *
 *  Created on: June 4, 2012
 *      Author: Ren Guanjiao
 */

#include "CamTagPositionSensor.h"

using namespace std;

//To open the camera and initialize camera, default -1 means opening random camera
CamPosiSensor::CamPosiSensor(int CamNum) {
  bResult = false;
  tagsNum = 0;
  for (int i = 0; i < 8; i++) {
    tagInfo_old[i] = 0;
  }

  //open cam
  capture = cvCaptureFromCAM(CamNum);
  if (!capture) {
    fprintf(stderr, "ERROR: capture is NULL.  Press keyboard to continue... \n");
    getchar(); //press keyboard to continue
  } else {
    //capture image
    frame = 0;
    if (!cvGrabFrame(capture)) {
      cout << "can not get image\n";
    } else {
      frame = cvRetrieveFrame(capture);

      // convert to gray image
      img = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);

      if (frame->nChannels == 3) {
        cvCvtColor(frame, img, CV_BGR2GRAY);
      } else {
        img = cvCloneImage(frame);
      }

      // connect to sever
      fflush(nullptr);
      //Client mylink(5010, -1, "localhost", false, &bResult);
      mylink = new Client(5010, -1, "localhost", false, &bResult);

      if (!bResult) {
        printf("Failed to create Client object! Press any key to continue... \n");
        getchar();
      } else {
        printf("Client, made connection...\n");
        fflush(nullptr);

        //transfer image info
        sprintf(cImgH, "%d\n", img->height); //The string must end with "\n"!
        sprintf(cImgW, "%d\n", img->width);
        printf("Client, Sending image info");
        fflush(nullptr);
        mylink->SendString(cImgH);
        fflush(nullptr);
        mylink->SendString(cImgW);
        //img=cvLoadImage("./Imgs/Image-1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
        cvNamedWindow("Client Image", 0);
      }
    }
  }

}

//To close the camera and release the memory
CamPosiSensor::~CamPosiSensor() {
  //close the port
  printf("Client, closing connection...\n");
  fflush(nullptr);
  if (bResult) {
    mylink->Close();
    delete mylink;
  }

  printf("Client, done...\n");
  fflush(nullptr);

  //close the window and release the image
  cvDestroyWindow("Client Image");
  cvReleaseImage(&img);

}

// -------- Capture one frame of the camera by Ren -------------
double* CamPosiSensor::CaptureFrame() {
  //capture frames
  if (!cvGrabFrame(capture)) {
    cout << "can not get image\n";
    return nullptr;
  }
  frame = cvRetrieveFrame(capture);

  //convert to gray
  if (frame->nChannels == 3) {
    cvCvtColor(frame, img, CV_BGR2GRAY);
  } else {
    img = cvCloneImage(frame);
  }

  char imgData[img->height * img->width + 1];
  sprintf(imgData, "%s", img->imageData);

  //transfer image
  //	printf("Client, Sending image data \n");
  fflush(nullptr);
  mylink->SendString(imgData);

  //Receive info of tags: number of tags
  fflush(nullptr);
  mylink->RecvString(cTagsNum, 255, '\n');
  tagsNum = atoi(cTagsNum);

  //	printf("Number of tags: %d \n", tagsNum);

  char cTagsInfo[255];

  for (int i = 0; i < tagsNum; i++) {
    fflush(nullptr);
    mylink->RecvString(cTagsInfo, 255, '\n');
  }

  if (tagsNum > 0 ) {
      sscanf(cTagsInfo, "%lf %lf %lf %lf %lf %lf %lf %lf",
             &tagInfo_old[0], &tagInfo_old[1], &tagInfo_old[2],
             &tagInfo_old[3], &tagInfo_old[4], &tagInfo_old[5],
             &tagInfo_old[6], &tagInfo_old[7]);
  }

  return tagInfo_old;
}

// ---------- Show the image to screen by Ren --------------
void CamPosiSensor::ShowImage() {
  //If not show image, just comment following two lines.
  cvShowImage("Client Image", frame);
  cvWaitKey(10);
}
