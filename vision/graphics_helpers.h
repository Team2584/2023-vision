#ifndef graphics_heplers_h
#define graphics_heplers_h

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <opencv2/opencv.hpp>

extern "C"
{
#include <apriltag/apriltag.h>
}

#define IMG_MARGIN 20

bool in_margin(double p[], int width, int height);
void labelDetections(cv::Mat frame, apriltag_detection_t *det);
void drawMargins(cv::Mat frame);

#endif
