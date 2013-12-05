/*
 * CamTagPositionSensor.cpp
 *
 *  Created on: June 4, 2012
 *      Author: Ren Guanjiao
 */

#include "CamTagPositionSensor.h"

using namespace std;

//To open the camera and initialize camera, default -1 means opening random camera
CamPosiSensor::CamPosiSensor(int CamNum = -1) {
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
      fflush(NULL);
      //Client mylink(5010, -1, "localhost", false, &bResult);
      mylink = new Client(5010, -1, "localhost", false, &bResult);

      if (!bResult) {
        printf("Failed to create Client object! Press any key to continue... \n");
        getchar();
      } else {
        printf("Client, made connection...\n");
        fflush(NULL);

        //transfer image info
        sprintf(cImgH, "%d\n", img->height); //The string must end with "\n"!
        sprintf(cImgW, "%d\n", img->width);
        printf("Client, Sending image info");
        fflush(NULL);
        mylink->SendString(cImgH);
        fflush(NULL);
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
  fflush(NULL);
  if (bResult) {
    mylink->Close();
    delete mylink;
  }

  printf("Client, done...\n");
  fflush(NULL);

  //close the window and release the image
  cvDestroyWindow("Client Image");
  cvReleaseImage(&img);

}

// -------- Capture one frame of the camera by Ren -------------
double* CamPosiSensor::CaptureFrame() {
  //capture frames
  if (!cvGrabFrame(capture)) {
    cout << "can not get image\n";
    return NULL;
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
  fflush(NULL);
  mylink->SendString(imgData);

  //Receive info of tags: number of tags
  fflush(NULL);
  mylink->RecvString(cTagsNum, 255, '\n');
  tagsNum = atoi(cTagsNum);

  //	printf("Number of tags: %d \n", tagsNum);

  char cTagsInfo[255];

  double tagsInfo[tagsNum][8]; //save all tag data to this array
  for (int i = 0; i < tagsNum; i++) {
    fflush(NULL);
    mylink->RecvString(cTagsInfo, 255, '\n');

    sscanf(cTagsInfo, "%lf %lf %lf %lf %lf %lf %lf %lf", &tagsInfo[i][0], &tagsInfo[i][1], &tagsInfo[i][2],
        &tagsInfo[i][3], &tagsInfo[i][4], &tagsInfo[i][5], &tagsInfo[i][6], &tagsInfo[i][7]);

    //		printf("Time = %f\n ID = %f\n XYZ = %.3f %.3f %.3f \n RPY = %.3f %.3f %.3f \n", tagsInfo[i][0], tagsInfo[i][1], tagsInfo[i][2], tagsInfo[i][3],
    //			    tagsInfo[i][4], tagsInfo[i][5], tagsInfo[i][6], tagsInfo[i][7]);

    for (int j = 0; j < 8; j++) {
      tagInfo_old[j] = tagsInfo[i][j];
    }
  }

  //only return one tag value
  if (0 == tagsNum) {
    return tagInfo_old;
  } else {
    return tagsInfo[tagsNum - 1];
  }
}

// ---------- Show the image to screen by Ren --------------
void CamPosiSensor::ShowImage() {
  //If not show image, just comment following two lines.
  cvShowImage("Client Image", frame);
  cvWaitKey(10);
}
