#ifndef pose_estimation_h
#define pose_estimation_h

#include <Eigen/Eigen>
#include <chrono>
#include <cmath>

extern "C"
{
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
}

#include "Cameras.h"

#define M_TWOPI 2 * M_PI

#define TAG_SIZE 0.1524
#define INCH 39.37

typedef struct tag_position
{
    double x;
    double y;
    double z;
    double theta;
} tag_position;

void getRobotPosition(apriltag_detection_t *det, robot_position *pos, camInfo *cam);

#endif
