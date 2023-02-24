#include "Cameras.h"

#define IMG_MARGIN 20

using namespace std;
using namespace cv;

void abstractCamera::setPosOffset(double x, double y, double z, double theta, double elevAngle)

{
    info.offset.x = x;
    info.offset.y = y;
    info.offset.z = z;
    info.offset.theta = theta;
    info.offset.elevAngle = elevAngle;
}

void abstractCamera::setDistCoeffs(double c0, double c1, double c2, double c3, double c4)
{
    info.distCoeffs[0] = c0;
    info.distCoeffs[1] = c1;
    info.distCoeffs[2] = c2;
    info.distCoeffs[3] = c3;
    info.distCoeffs[4] = c4;
}

void abstractCamera::setCamParams(double fx, double fy, double cx, double cy)
{
    info.camMatx << 0, 0, 0, 0, 0, 0, 0, 0, 1;
    info.camMatx(0, 0) = fx;
    info.camMatx(1, 1) = fy;
    info.camMatx(0, 2) = cx;
    info.camMatx(1, 2) = cy;
}
