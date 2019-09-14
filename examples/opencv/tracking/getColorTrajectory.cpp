/*
 * getColorTrajectory.cpp
 *
 *  Created on: Oct 18, 2015
 *      Author: giuliano
 */

#include "getColorTrajectory.h"

getColorTrajectory::getColorTrajectory() {
	// TODO Auto-generated constructor stub

}

getColorTrajectory::~getColorTrajectory() {
	// TODO Auto-generated destructor stub
}

void getColorTrajectory::setColor(int hMin, int sMin, int vMin, int hMax, int sMax, int vMax)
{
	colorDetected.push_back(hMin);
	colorDetected.push_back(sMin);
	colorDetected.push_back(vMin);
	colorDetected.push_back(hMax);
    colorDetected.push_back(sMax);
    colorDetected.push_back(vMax);
}

void  getColorTrajectory::setLineColor(int r, int g, int b)
{
	lineRGB.push_back(r);
	lineRGB.push_back(g);
	lineRGB.push_back(b);
}



vector<Point2f> getColorTrajectory::getTrajectory()
{
	return trajectory;
}


Mat getColorTrajectory::getFrame()
{
	return resized;
}

vector<int>  getColorTrajectory::getLineColor()
{
	return lineRGB;
}


void getColorTrajectory::detectTrajectory(Mat frame)
{


			resized=frame;
	        cv::Mat hsv_image, detected;
	        cv::medianBlur(resized, resized, 3);//blur the image before changing to HSV
	        cv::cvtColor(resized, hsv_image, cv::COLOR_BGR2HSV);//change from BGR to HSV, easier to detect colors


	        cv::inRange(hsv_image, cv::Scalar(colorDetected.at(0), colorDetected.at(1), colorDetected.at(2)),//detect a color
	        cv::Scalar(colorDetected.at(3), colorDetected.at(4), colorDetected.at(5)), detected);

	        cv::medianBlur(detected, detected,3);//blur again



	        Mat canny_output;
		    vector<vector<Point> > contours;
		    vector<Vec4i> hierarchy;
		    RNG rng(12345);

		    /// Detect edges using canny
		    Canny( detected, canny_output, 100, 100*2, 3 );
		    /// Find contours
		    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	    	  /// Draw contours
		    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
		    for( int i = 0; i< contours.size(); i++ )
		  	 {
		  	     Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		  	     drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
			 }


		      //saves the contours
	          findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	          /// Get the moments
	          vector<Moments> mu(contours.size() );
	          if(contours.empty()==false)
	    {
	          for( int i = 0; i < contours.size(); i++ )
	          { mu[i] = moments( contours[i], false ); }

	          ///  Get the mass centers:
	          vector<Point2f> mc( contours.size() );

	          for( int i = 0; i < contours.size(); i++ )
	           { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

	          if((mc[0].x != mc[0].x) == false)//VERY IMPORTANT!!! THIS line delete all the NaN value from the trajectory
	          trajectory.push_back(mc[0]);//only save one contour


	    }


}


void getColorTrajectory::drawTrajectory(int r, int g, int b)
{
	 if(trajectory.size()>=2)
	 {
		 for(int i=1;i<trajectory.size();i++)
	     line(resized, trajectory.at(i-1),trajectory.at(i), Scalar(r,g,b), 3, 8, 0);
	 }
}
