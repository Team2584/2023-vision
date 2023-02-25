#include "graphics_helpers.h"

using namespace std;
using namespace cv;

bool in_margin(double p[], int width, int height)
{
    if (p[0] < IMG_MARGIN || p[0] > width - IMG_MARGIN)
        return true;
    if (p[1] < IMG_MARGIN || p[1] > height - IMG_MARGIN)
        return true;
    return false;
}

void labelDetections(Mat frame, apriltag_detection_t *det)
{

    // Draw detection outline
    line(frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]),
         Scalar(0xff, 0x00, 0x00), 2);
    line(frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[3][0], det->p[3][1]),
         Scalar(0x00, 0xff, 0x00), 2);
    line(frame, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]),
         Scalar(0x00, 0x00, 0xff), 2);
    line(frame, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]),
         Scalar(0xff, 0x00, 0xff), 2);

    // Label the tag
    stringstream ss;
    ss << det->id;
    String text = ss.str();
    int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontscale = 1.0;
    int baseline;
    Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
    putText(frame, text, Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2),
            fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
}

void drawMargins(Mat frame)
{
    int cols = frame.cols;
    int rows = frame.rows;
    line(frame, Point(IMG_MARGIN, rows - IMG_MARGIN), Point(cols - IMG_MARGIN, rows - IMG_MARGIN),
         Scalar(0xff, 0xff, 0xff), 2);
    line(frame, Point(IMG_MARGIN, rows - IMG_MARGIN), Point(IMG_MARGIN, IMG_MARGIN),
         Scalar(0xff, 0xff, 0xff), 2);
    line(frame, Point(cols - IMG_MARGIN, rows - IMG_MARGIN), Point(cols - IMG_MARGIN, IMG_MARGIN),
         Scalar(0xff, 0xff, 0xff), 2);
    line(frame, Point(IMG_MARGIN, IMG_MARGIN), Point(cols - IMG_MARGIN, IMG_MARGIN),
         Scalar(0xff, 0xff, 0xff), 2);
}
