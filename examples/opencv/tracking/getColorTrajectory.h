/*
 * getColorTrajectory.h
 *
 *  Created on: Oct 18, 2015
 *      Author: giuliano
 */

#ifndef GETCOLORTRAJECTORY_H_
#define GETCOLORTRAJECTORY_H_

#include <iostream>
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <string>
#include <vector>
#include <fstream>


using namespace std;
using namespace cv;


class getColorTrajectory {
public:
	getColorTrajectory();
	virtual ~getColorTrajectory();
	void setColor(int hMin, int sMin, int vMin, int hMax, int sMax, int vMax);
	void drawTrajectory(int r, int g, int b);
	void saveToFile(string path);
	void detectTrajectory(Mat frame);
	Mat getFrame();
	void setLineColor(int r, int g, int b);
	vector<int> getLineColor();
	vector<Point2f> getTrajectory();
private:
	vector<int> colorDetected, lineRGB;
	vector<Point2f> trajectory;
	Mat resized;

};

#endif /* GETCOLORTRAJECTORY_H_ */
