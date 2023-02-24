#ifndef main_h
#define main_h

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/RawTopic.h>
#include <networktables/StringTopic.h>

#include <chrono>

#include "nadjieb/mjpeg_streamer.hpp"

extern "C"
{
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
}

// AprilTag Parameters
#define HAMM_HIST_MAX 10
#define HAMMING_NUMBER 1
#define QUAD_DECIMATE 2.0
#define QUAD_SIGMA 0.0
#define NTHREADS 4
#define APRIL_DEBUG 0
#define REFINE_EDGES 1

// Camera parameters
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480

#define IMG_MARGIN 20

// Helper functions in main
double time_since(std::chrono::time_point<std::chrono::steady_clock> start);

#endif
