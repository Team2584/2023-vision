#ifndef detection_h
#define detection_h

#include "Cameras.h"
#include "graphics_helpers.h"
#include "pose_estimation.h"
#include <apriltag/apriltag.h>
#include <opencv2/opencv.hpp>

std::vector<robot_position> getPoses(cv::Mat grayFrame, cv::Mat colorFrame, camInfo *cam,
                                     apriltag_detector_t *td);

#endif
