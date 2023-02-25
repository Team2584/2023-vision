#include "Cameras.h"
#include "globals.h"
#include <chrono>

using namespace std;
using namespace cv;

depthCamera::depthCamera(string camSerial, int width, int height, int fps)
    : pipe{}, cfg{}, prof{}, align{RS2_STREAM_COLOR}, rs_colorFrame{nullptr},
      rs_depthFrame{nullptr}, colorFrame{cv::Size(width, height), CV_8UC3},
      grayFrame{cv::Size(width, height), CV_8UC1}, depthFrame{cv::Size(width, height), CV_16UC1}
{
    this->width = width;
    this->height = height;

    // Select camera by serial number
    cfg.enable_device(camSerial);

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);

    // Instruct pipeline to start streaming with the requested configuration
    prof = pipe.start(cfg);

    // Disgusting one-liner to turn on/off laser
    prof.get_device().first<rs2::depth_sensor>().set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
}

depthCamera::~depthCamera()
{
}

void depthCamera::setManualExposure(int exposuretime)
{
    // TODO impl
}

void depthCamera::setAutoExposure()
{
    // TODO impl
}

void depthCamera::getFrame()
{
    rs2::frameset frames;
    frames = pipe.wait_for_frames();

    frames = align.process(frames);

    rs_colorFrame = frames.get_color_frame();
    rs_depthFrame = frames.get_depth_frame();

    colorFrame.data = (uint8_t *)rs_colorFrame.get_data();
    cv::cvtColor(colorFrame, grayFrame, cv::COLOR_BGR2GRAY);

    depthFrame = Mat(Size(width, height), CV_16UC1, (uint16_t *)rs_depthFrame.get_data());
    depthUnit = rs_depthFrame.get_units();
}

bool isCone(vector<Point> contour, Rect boundBox, int width, int height)
{
    if (boundBox.width < 10 || boundBox.height < 10) // Discard small detections
        return false;

    if (boundBox.x < 2 || boundBox.x + boundBox.width > width - 2 || boundBox.y < 2)
        return false;

    vector<Point> hull;
    convexHull(contour, hull);
    vector<Point> approx;
    double epsCoeff = 0.03;
    double eps = epsCoeff * arcLength(contour, true);
    approxPolyDP(hull, approx, eps, true);
    if (approx.size() > 5)
        return false;

    return true;
}

double getObjectDepth(vector<Point> objContour, Mat depths)
{
    // Get depth by averaging depths of an eroded cone mask
    Mat objMask = Mat::zeros(Size(depths.cols, depths.rows), CV_8UC1);
    drawContours(objMask, vector<vector<Point>>(1, objContour), -1, 255, -1);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(objMask, objMask, kernel, Point(-1, -1), 3);

    /*
    Mat sel_depths;
    bitwise_and(depths, objMask, sel_depths);
    double sum = (double)cv::sum(sel_depths)[0];

    double depth = sum / countNonZero(sel_depths);
    */
    double depth = mean(depths, objMask)[0];

    return depth;
}

std::pair<double, double> depthCamera::findCones()
{
    Mat hsvFrame;
    GaussianBlur(this->colorFrame, hsvFrame, Size(17, 17), 1.2, 1.2, BORDER_DEFAULT);
    cvtColor(colorFrame, hsvFrame, COLOR_BGR2HSV);

    Mat mask;
    inRange(hsvFrame, Scalar(cone_min_hue, cone_min_sat, cone_min_val),
            Scalar(cone_max_hue, cone_max_sat, cone_max_val), mask);

    // Get contours of cones
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Show contours & bounding boxes
    vector<Rect> boundBoxes;
    vector<vector<Point>> coneContours;
    for (unsigned int i = 0; i < contours.size(); i++)
    {
        Rect boundBox = boundingRect(contours[i]);
        if (isCone(contours[i], boundBox, width, height))
        {
            drawContours(colorFrame, contours, i, Scalar(0, 255, 255), 2);
            rectangle(colorFrame, boundBox, Scalar(255, 255, 255), 1);
            coneContours.push_back(contours[i]);
            boundBoxes.push_back(boundBox);
        }
    }

    if (boundBoxes.size() == 0)
        return pair(0, 0);

    // Pick the largest cone
    double area;
    int selected = 0;
    for (unsigned int i = 0; i < coneContours.size(); i++)
    {
        double newArea = boundBoxes[i].width * boundBoxes[i].height;
        if (newArea > area)
        {
            area = newArea;
            selected = i;
        }
    }

    vector<Point> bestContour = coneContours[selected];
    double bestDist = getObjectDepth(bestContour, depthFrame) * depthUnit;
    drawContours(colorFrame, vector<vector<Point>>(1, bestContour), -1, Scalar(0, 255, 0), 7);

    Rect bestBox = boundBoxes[selected];
    rectangle(colorFrame, bestBox, Scalar(255, 255, 255), 2);

    // Mark the center
    Moments M = moments(bestContour);
    int bestCenterX = (int)(M.m10 / M.m00);
    int bestCenterY = (int)(M.m01 / M.m00);
    line(colorFrame, Point(bestCenterX - 5, bestCenterY), Point(bestCenterX + 5, bestCenterY),
         Scalar(0, 255, 255), 1);
    line(colorFrame, Point(bestCenterX, bestCenterY + 5), Point(bestCenterX, bestCenterY - 5),
         Scalar(0, 255, 255), 1);

    double XpxFromCenter = (bestCenterX - (width / 2));
    double YpxFromCenter = ((height / 2) - bestCenterY);
    // The 0.108 is a camera property: FOV_in_degrees / frame_width
    // Same goes for the vertical one
    double xAngle = XpxFromCenter * 0.108;
    double yAngle = YpxFromCenter * 0.089;
    // cout << "angle: " << yAngle << endl;
    xAngle *= M_PI / 180;
    yAngle *= M_PI / 180;
    // Convert from spherical coordinates to cartesian
    double x = bestDist * cos(yAngle) * sin(xAngle);
    double y = bestDist * cos(yAngle) * cos(xAngle);
    double z = bestDist * sin(yAngle);

    // Compensate for the camera being tilted down by rotating the vector
    Eigen::Vector3d pos;
    pos << x, y, z;

    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << 1, 0, 0, 0, cos(info.offset.elevAngle), -sin(info.offset.elevAngle), 0,
        sin(info.offset.elevAngle), cos(info.offset.elevAngle);

    pos = rotationMatrix * pos;

    x = pos(0) + info.offset.x;
    y = pos(1) + info.offset.y;
    z = pos(2) + info.offset.z;

    // cout << "Cone X: " << x << endl;
    // cout << "Cone Y: " << y << endl;
    // cout << "Cone Z: " << z << endl;

    return pair(x, y);
}

std::pair<double, double> depthCamera::findCubes()
{
    Mat hsvFrame;
    GaussianBlur(colorFrame, hsvFrame, Size(17, 17), 1.2, 1.2, BORDER_DEFAULT);

    cvtColor(colorFrame, hsvFrame, COLOR_BGR2HSV);

    Mat mask;
    inRange(hsvFrame, Scalar(cube_min_hue, cube_min_sat, cube_min_val),
            Scalar(cube_max_hue, cube_max_sat, cube_max_val), mask);

    // Morphological open to get rid of nonsense
    Mat morphedMask;
    Mat openKernel = getStructuringElement(MORPH_RECT, Size(6, 6));
    morphologyEx(mask, morphedMask, MORPH_OPEN, openKernel);

    Mat closeKernel = getStructuringElement(MORPH_RECT, Size(30, 30));
    morphologyEx(morphedMask, morphedMask, MORPH_CLOSE, closeKernel);

    // imshow("mask", morphedMask);

    // Get contours of cones
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(morphedMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Show contours & bounding boxes
    vector<Rect> boundBoxes;
    vector<vector<Point>> goodContours;
    for (unsigned int i = 0; i < contours.size(); i++)
    {
        Rect boundBox = boundingRect(contours[i]);
        // if (boundBox.width < 10 || boundBox.height < 10) // Discard small detections
        // continue;

        goodContours.push_back(contours[i]);
        boundBoxes.push_back(boundBox);
        drawContours(colorFrame, contours, i, Scalar(255, 255, 0), 2, FILLED);
        rectangle(colorFrame, boundBox, Scalar(255, 0, 0), 1);
    }

    // Get convex hulls of contours
    vector<vector<Point>> hulls(goodContours.size());
    for (uint16_t i = 0; i < goodContours.size(); i++)
    {
        convexHull(contours[i], hulls[i]);
        // drawContours(colorFrame, hulls, i, Scalar(0, 0, 255), 2, FILLED);
    }

    // Approximate contours
    vector<vector<Point>> approx(contours.size());
    double epsCoeff = 0.10;
    for (uint16_t i = 0; i < contours.size(); i++)
    {
        double eps = epsCoeff * arcLength(contours[i], true);
        approxPolyDP(hulls[i], approx[i], eps, true);
        // if (approx[i].size() <= 6)
        // drawContours(colorFrame, approx, i, Scalar(0, 0, 255), 2, FILLED);
    }

    // If there are no cones
    if (boundBoxes.size() == 0)
        return pair(0, 0);

    // For now just select largest cone detection
    int area = 0;
    int largest = 0;
    for (unsigned int i = 0; i < boundBoxes.size(); i++)
    {
        if (boundBoxes[i].width * boundBoxes[i].height > area)
        {
            area = boundBoxes[i].width * boundBoxes[i].height;
            largest = i;
        }
    }

    vector<Point> cubeContour = approx[largest];
    Rect cube = boundBoxes[largest];

    int cubeCenterX = cube.x + (cube.width / 2);
    int cubeCenterY = cube.y + (cube.height / 2);

    line(colorFrame, Point(cubeCenterX - 5, cubeCenterY), Point(cubeCenterX + 5, cubeCenterY),
         Scalar(255, 255, 255), 1);
    line(colorFrame, Point(cubeCenterX, cubeCenterY + 5), Point(cubeCenterX, cubeCenterY - 5),
         Scalar(255, 255, 255), 1);

    Mat depthRoi(depthFrame, cube);
    Mat cubeMask = Mat::zeros(Size(width, height), CV_8UC1);
    drawContours(cubeMask, vector<vector<Point>>(1, cubeContour), -1, 255, -1);

    double distAvg = mean(depthFrame, cubeMask)[0] * depthUnit;

    cout << "Distance: " << distAvg * INCH << endl;

    double XpxFromCenter = (cubeCenterX - (width / 2));
    double YpxFromCenter = ((height / 2) - cubeCenterY);
    // The 0.108 is a camera property: FOV_in_degrees / frame_width
    // Same goes for the vertical one
    double xAngle = XpxFromCenter * 0.108;
    double yAngle = YpxFromCenter * 0.089;
    // cout << "angle: " << yAngle << endl;
    xAngle *= M_PI / 180;
    yAngle *= M_PI / 180;
    // Convert from spherical coordinates to x, y, z
    double x = distAvg * cos(yAngle) * sin(xAngle);
    double y = distAvg * cos(yAngle) * cos(xAngle);
    double z = distAvg * sin(yAngle);

    // Compensate for the camera being tilted down by rotating the vector
    Eigen::Vector3d pos;
    pos << x, y, z;

    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << 1, 0, 0, 0, cos(info.offset.elevAngle), -sin(info.offset.elevAngle), 0,
        sin(info.offset.elevAngle), cos(info.offset.elevAngle);

    pos = rotationMatrix * pos;

    x = pos(0) + info.offset.x;
    y = pos(1) + info.offset.y;
    z = pos(2) + info.offset.z;

    // cout << "Cube X: " << x << endl;
    // cout << "Cube Y: " << y << endl;
    // cout << "Cube Z: " << z << endl;

    return pair(x, y);
}

std::pair<double, double> depthCamera::findPoles()
{
    /*
    Mat depthData(Size(width, height), CV_16UC1, (uint16_t *)depthFrame.get_data());
    double depthUnit = depthFrame.get_units();

    Mat depthEdges;
    Canny(depthData, edges, CV_64F, 1, 1);

    Mat edgesView;
    edges.convertTo(edgesView, CV_8UC1);

    imshow("depth edges", edges);
    */
    return pair(0, 0);
}
