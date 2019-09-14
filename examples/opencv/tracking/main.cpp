/*
 * main.cpp
 *
 *  Created on: Oct 18, 2015
 *      Author: giuliano
 */
#include "getColorTrajectory.h"
#include<sstream>
//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/highgui/imgproc.hpp>
//#include <opencv2/imgcodecs.hpp>

int main(int argc, char* argv[])
{

  bool endVideo=false;
  vector<int> line1, line2, line3;
  string path1_x,path2_x,path3_x,path1_y,path2_y,path3_y;
  string finalFrameColor1, finalFrameColor2, finalFrameColor3, finalFrameAllColors;
  VideoCapture cap("ExampleVideo.mp4"); // open the video file for reading

  ofstream color1Trajectory_x,color2Trajectory_x,color3Trajectory_x,color1Trajectory_y,color2Trajectory_y,color3Trajectory_y;
  getColorTrajectory color1, color2,color3;
  int countFrames=0;;
  stringstream streamColor1, streamColor2, streamColor3, streamAllColors;
  string filenameCol1, filenameCol2, filenameCol3, filenameAllColors;
  //SETTINGS//

  Size outputFrameSize(750,450);
  //output frame size
  //choose the size of your image. This is very important.
  //if you set a small size, the contours detected will be too small
  //and therefore the centroids can't be computed.

  bool saveLastFrame = true;
  //true:youwill save the last frame as picture, you will have the
  //picture with the final trajectory
  //if trajectoriesInOneFrame is false, you will only have 3 pictures
  //with individual trajectories
  //if trajectoriesInOneFrame is true, you will have 3 pictures
  //with individual trajectories and one picture with all the trajectories

  //false: dont's save the final frame
  finalFrameColor1="Color1Trajectory";
  finalFrameColor2="Color2Trajectory";
  finalFrameColor3="Color3TRajectory";
  finalFrameAllColors="Trajectories";



  bool showVideo = false;
  //show video
  //the video that you updated will be shown
  //even if nothing is detected the video will still be shown


  bool trajectoriesInOneFrame = false;
  //trajectories in one frame
  //false: each trajectory will be in one individual window
  //true:  each trajectory will be in one individual window, one additional window will
  //show all the trajectories together

  bool printTrajectoriesToFiles=true;
  // print trajectories to file
  // false: trajectories will not be saved
  // true: trajectories will be saved in a file, you can set the path

  //choose the path where you want your trajectories to be saved
  //will be printed one file per each coordinate per trajectory
  //the standard path will save the trajectories in the project folder
  //remember to reload the folder(f5 on the folder name) to see the files
  path1_x="color1_x.txt";
  path2_x="color2_x.txt";
  path3_x="color3_x.txt";
  path1_y="color1_y.txt";
  path2_y="color2_y.txt";
  path3_y="color3_y.txt";


  //which colors you want to detect???
  //here you need to add minimum and maximum HSV values.
  //In order to get these values do the following:
  //Donwload gimp for Ubuntu, and open the toolbox
  //Select a color in the color box
  //The color will have an H value=x
  //then your minimum and max value for H will be (x/2)-10 and (x/2)+10 respectively
  //the other values are not VERY important, is ok to tune them manually..
  //EXAMPLE..H value RED in Gimp: 360
  //minH=(360/2)-10=170 maxH=(360/2)+10=190
  color1.setColor(170,100,160,190,255,255);//red
  color2.setColor(15,100,100,40,255,255);//yellow
  color3.setColor(110,100,100,130,255,255);//blue
  //color3.setColor(50,100,100,70,255,255);//green

  //Which color you want to use for the trajectory?
  //Of course the best would be to use the same color as the detected one
  //but you can change it if you want. Here you have to useRGB values
  //you can use Gimp agin to detecg RGB values.
  //PLEASE NOTE..in OpenCV a color is defined as Scalar(a,b,c)
  //where a=Blue b=Green c=Red
  //Example Red in Gimp = 255 0 0
  //Red in Opencv Scalar(0,0,255).. use this definition to set your color line
  color1.setLineColor(0,0,255);//red
  color2.setLineColor(0,255,255);//yellow
  color3.setLineColor(255,0,0);//blue
  //color3.setLineColor(0,255,0);//green


  //SETTINGS

  if(saveLastFrame==true)
  {

    string type = ".jpg";

    streamColor1<<finalFrameColor1<<type;
    filenameCol1 = streamColor1.str();
    streamColor1.str("");




    streamColor2<<finalFrameColor2<<type;
    filenameCol2 = streamColor2.str();
    streamColor2.str("");




    streamColor3<<finalFrameColor3<<type;
    filenameCol3 = streamColor3.str();
    streamColor3.str("");

    if(trajectoriesInOneFrame==true)
    {


      streamAllColors<<finalFrameAllColors<<type;
      filenameAllColors = streamAllColors.str();
      streamAllColors.str("");

    }


  }





  if ( !cap.isOpened() )
  {
    cout << "Cannot open the video file" << endl;
    return -1;
  }

  double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video

  while(1)
  {

    Mat frameColor1,frameColor2,frameColor3, frame, totalFrame, showVideoFrame;



    bool bSuccess = cap.read(frame); // read a new frame from video


    if(frame.empty())
    {

      cout<< "endddddd";
      break;
    }

    if (!bSuccess) //if not success, break loop
    {
      cout << "Cannot read the frame from video file" << endl;
      break;
    }


    resize(frame, frame, outputFrameSize, 0, 0);//set image size

    frameColor1=frame.clone();
    frameColor2=frame.clone();
    frameColor3=frame.clone();
    totalFrame=frame.clone();
    showVideoFrame=frame.clone();




    //Drawing trajectories in individual frames
    color1.detectTrajectory(frameColor1);
    color1.drawTrajectory(color1.getLineColor().at(0),color1.getLineColor().at(1),color1.getLineColor().at(2));

    color2.detectTrajectory(frameColor2);
    color2.drawTrajectory(color2.getLineColor().at(0),color2.getLineColor().at(1),color2.getLineColor().at(2));

    color3.detectTrajectory(frameColor3);
    color3.drawTrajectory(color3.getLineColor().at(0),color3.getLineColor().at(1),color3.getLineColor().at(2));


    //Drawing trajectories in one frame
    if(color1.getTrajectory().size()>=2)
    {
      for(int i=1;i<color1.getTrajectory().size();i++)
        line(totalFrame, color1.getTrajectory().at(i-1),color1.getTrajectory().at(i), Scalar(color1.getLineColor().at(0),color1.getLineColor().at(1),color1.getLineColor().at(2)), 3, 8, 0);
    }

    if(color2.getTrajectory().size()>=2)
    {
      for(int i=1;i<color2.getTrajectory().size();i++)
        line(totalFrame, color2.getTrajectory().at(i-1),color2.getTrajectory().at(i), Scalar(color2.getLineColor().at(0),color2.getLineColor().at(1),color2.getLineColor().at(2)), 3, 8, 0);
    }

    if(color3.getTrajectory().size()>=2)
    {
      for(int i=1;i<color3.getTrajectory().size();i++)
        line(totalFrame, color3.getTrajectory().at(i-1),color3.getTrajectory().at(i), Scalar(color3.getLineColor().at(0),color3.getLineColor().at(1),color3.getLineColor().at(2)), 3, 8, 0);
    }


    if(saveLastFrame==true)
    {
      if(color1.getTrajectory().size()>0)
        imwrite(filenameCol1,frameColor1);

      if(color2.getTrajectory().size()>0)
        imwrite(filenameCol2,frameColor2);

      if(color3.getTrajectory().size()>0)
        imwrite(filenameCol3,frameColor3);

      if(trajectoriesInOneFrame==true)
      {
        imwrite(filenameAllColors,totalFrame);

      }


    }


    if(trajectoriesInOneFrame==true)
      imshow("Trajectories", totalFrame);

    if(color1.getTrajectory().size()> 0)
      imshow("Color1", color1.getFrame());

    if(color2.getTrajectory().size()> 0)
      imshow("Color2", color2.getFrame());

    if(color3.getTrajectory().size()> 0)
      imshow("Color3", color3.getFrame());

    if(showVideo==true)
      imshow("Video", showVideoFrame);

    if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
    {
      cout << "esc key is pressed by user" << endl;
      break;
    }
  }


  if(printTrajectoriesToFiles==true)
  {

    if(color1.getTrajectory().size()>0)
    {
      color1Trajectory_x.open(path1_x.c_str());
      color1Trajectory_y.open(path1_y.c_str());
      for(int i=0;i<color1.getTrajectory().size();i++)
      {
        color1Trajectory_x << color1.getTrajectory().at(i).x << endl;
        color1Trajectory_y << color1.getTrajectory().at(i).y << endl;
      }

      color1Trajectory_x.close();
      color1Trajectory_y.close();
    }




    if(color2.getTrajectory().size()>0)
    {
      color2Trajectory_x.open(path2_x.c_str());
      color2Trajectory_y.open(path2_y.c_str());
      for(int i=0;i<color2.getTrajectory().size();i++)
      {
        color2Trajectory_x << color2.getTrajectory().at(i).x << endl;
        color2Trajectory_y << color2.getTrajectory().at(i).y << endl;
      }

      color2Trajectory_x.close();
      color2Trajectory_y.close();
    }


    if(color3.getTrajectory().size()>0)
    {
      color3Trajectory_x.open(path3_x.c_str());
      color3Trajectory_y.open(path3_y.c_str());
      for(int i=0;i<color3.getTrajectory().size();i++)
      {
        color3Trajectory_x << color3.getTrajectory().at(i).x << endl;
        color3Trajectory_y << color3.getTrajectory().at(i).y << endl;
      }

      color3Trajectory_x.close();
      color3Trajectory_y.close();
    }


  }




  return 0;

}




