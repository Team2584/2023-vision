#include "Cameras.h"

using namespace std;
using namespace cv;

usbCamera::usbCamera(int camNum, int width, int height, int fps)
    : cap{camNum}, colorFrame{cv::Size(width, height), CV_8UC3}, grayFrame{cv::Size(width, height),
                                                                           CV_8UC1}
{
    setDistCoeffs();
    if (cap.isOpened())
    {
        cap.set(CAP_PROP_FRAME_WIDTH, width);
        cap.set(CAP_PROP_FRAME_HEIGHT, height);
        cap.set(CAP_PROP_FPS, fps);
    }
}

usbCamera::~usbCamera()
{
}

void usbCamera::setManualExposure(int exposuretime)
{
    // TODO impl
}

void usbCamera::setAutoExposure()
{
    // TODO impl
}

void usbCamera::setManualFocus()
{
    // TODO impl
}

void usbCamera::setAutoFocus()
{
    // TODO impl
}

void usbCamera::getFrame()
{
    cap >> colorFrame;
    cvtColor(colorFrame, grayFrame, COLOR_BGR2GRAY);
}
