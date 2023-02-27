#ifndef Camera_h
#define Camera_h

#include <Eigen/Eigen>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>
#include <apriltag/apriltag.h>
#include <chrono>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

// Aliases for depth camera serial numbers; color-coded with tape on each physical camera
#define DEPTH_BLUE "017322071728"
#define DEPTH_RED "939622072805"
#define DEPTH_YELLOW "025222072169"

#define INCH 39.37

typedef struct camPosOffset
{
    double x;
    double y;
    double z;
    double theta;
    double elevAngle;
} camPosOffset;

typedef struct camInfo
{
    // Camera offset from robot center
    camPosOffset offset;

    /**********
     * Camera matrix containing camera intrinsics, vector of destortion coefficients of the camera.
     * For details on what these are, see the opencv camera calibration docs:
     * https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html
     * All these values come from the scripts in ../camera-calibration
     **********/
    cv::Vec<double, 5> distCoeffs;
    cv::Matx33d camMatx;
} camInfo;

typedef struct robot_position
{
    double x;
    double y;
    double z;
    double theta;           // in radians
    double processing_time; // in microseconds
} robot_position;

class abstractCamera
{
  protected:
    int width;
    int height;
    int fps;

  public:
    camInfo info;
    cv::Mat colorFrame;
    cv::Mat grayFrame;
    std::chrono::time_point<std::chrono::steady_clock> frameTime;

    virtual void getFrame() = 0;
    virtual void setManualExposure(int exposuretime) = 0;
    virtual void setAutoExposure() = 0;

    void setPosOffset(double x = 0, double y = 0, double z = 0, double theta = 0,
                      double elevAngle = 0);
    void setDistCoeffs(double c0 = 0, double c1 = 0, double c2 = 0, double c3 = 0, double c4 = 0);
    void setCamParams(double fx, double fy, double cx, double cy);
};

class flirCamera : public abstractCamera
{
  private:
    // Flir camera globals
    Spinnaker::SystemPtr flirSystem = Spinnaker::System::GetInstance();
    Spinnaker::CameraList flirCamList = flirSystem->GetCameras();
    Spinnaker::CameraPtr pCam = nullptr;
    int width = 720;
    int height = 540;

  public:
    camInfo info;
    cv::Mat colorFrame;
    cv::Mat grayFrame;

    flirCamera(int camNum);
    ~flirCamera();

    void setManualGain(double value);
    void setAutoGain();
    void setManualExposure(int exposuretime);
    void setAutoExposure();
    void getFrame();
};

class depthCamera : public abstractCamera
{
  private:
    // Create a pipeline which abstracts the camera
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default
    // profile
    rs2::config cfg;
    rs2::pipeline_profile prof;
    rs2::align align;
    rs2::video_frame rs_colorFrame;
    rs2::depth_frame rs_depthFrame;
    double depthUnit;

  public:
    cv::Mat colorFrame;
    cv::Mat grayFrame;
    cv::Mat depthFrame;

    depthCamera(std::string camSerial, int width, int height, int fps);
    ~depthCamera();

    void setManualExposure(int exposuretime);
    void setAutoExposure();
    void setLaser(bool on);
    void getFrame();
    std::pair<double, double> findCones();
    std::pair<double, double> findCubes();
    std::pair<double, double> findPoles();
};

class usbCamera : public abstractCamera
{
  private:
    cv::VideoCapture cap;

  public:
    cv::Mat colorFrame;
    cv::Mat grayFrame;

    usbCamera(int camNum, int width, int height, int fps);
    ~usbCamera();

    void setManualExposure(int exposuretime);
    void setAutoExposure();
    void setManualFocus();
    void setAutoFocus();
    void getFrame();
};

#endif
