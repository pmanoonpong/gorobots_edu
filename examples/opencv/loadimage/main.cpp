/*
 * Test.cpp
 *
 *  Created on: Oct 17, 2015
 *      Author: poma
 */
#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/highgui/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
//#include <opencv/cv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{


    Mat image;
    image = cv::imread("faceDetection.png", CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}
