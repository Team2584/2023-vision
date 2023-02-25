#include "nadjieb/mjpeg_streamer.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

#define WIDTH 640
#define HEIGHT 480
#define FPS 60

#define DEPTH_BLUE "017322071728"
#define DEPTH_RED "939622072805"

using namespace std;
using namespace cv;
using mjpgs = nadjieb::MJPEGStreamer;

int main(void)
{
    rs2::pipeline pipe;
    rs2::config cfg;

    // Select camera by serial number
    cfg.enable_device(DEPTH_RED);

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);

    // Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile prof = pipe.start(cfg);

    // Disgusting one-liner to disable laser
    prof.get_device().first<rs2::depth_sensor>().set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    vector<int> stream_params = {IMWRITE_JPEG_QUALITY, 90};
    mjpgs streamer;
    streamer.start(8080);

    int cone_min_hue;
    int cone_max_hue;
    int cone_min_sat;
    int cone_max_sat;
    int cone_min_val;
    int cone_max_val;

    int cube_min_hue;
    int cube_max_hue;
    int cube_min_sat;
    int cube_max_sat;
    int cube_min_val;
    int cube_max_val;


    while (true)
    {
        // Get frame
        rs2::frameset frames;
        frames = pipe.wait_for_frames();
        rs2::video_frame frame = frames.get_color_frame();
        Mat colorFrame(Size(WIDTH, HEIGHT), CV_8UC3, (void *)frame.get_data());
        GaussianBlur(colorFrame, colorFrame, Size(17, 17), 1.2, 1.2, BORDER_DEFAULT);

        // Get parameters
        vector<int> coneParams;
        vector<int> cubeParams;

        string line;
        ifstream coneFile("cone-params.txt");
        for (int i = 0; i < 6; i++)
        {
            if (coneFile)
                getline(coneFile, line);
            cout << line << endl;
            if (line != "")
                coneParams.push_back(stoi(line));
        }
        if (coneParams.size() > 0)
        {
            cone_min_hue = coneParams[0];
            cout << "cone_min_hue " << cone_min_hue << endl;
            cone_max_hue = coneParams[1];
            cout << "cone_max_hue " << cone_max_hue << endl;
            cone_min_sat = coneParams[2];
            cout << "cone_min_sat " << cone_min_sat << endl;
            cone_max_sat = coneParams[3];
            cout << "cone_max_sat " << cone_max_sat << endl;
            cone_min_val = coneParams[4];
            cout << "cone_min_val " << cone_min_val << endl;
            cone_max_val = coneParams[5];
            cout << "cone_max_val " << cone_max_val << endl << endl;
        }

        ifstream cubeFile("cube-params.txt");
        for (int i = 0; i < 6; i++)
        {
            if (coneFile)
                getline(cubeFile, line);
            cout << line << endl;
            if (line != "")
                cubeParams.push_back(stoi(line));
        }
        if (cubeParams.size() > 0)
        {
            cube_min_hue = cubeParams[0];
            cout << "cube_min_hue " << cube_min_hue << endl;
            cube_max_hue = cubeParams[1];
            cout << "cube_max_hue " << cube_max_hue << endl;
            cube_min_sat = cubeParams[2];
            cout << "cube_min_sat " << cube_min_sat << endl;
            cube_max_sat = cubeParams[3];
            cout << "cube_max_sat " << cube_max_sat << endl;
            cube_min_val = cubeParams[4];
            cout << "cube_min_val " << cube_min_val << endl;
            cube_max_val = cubeParams[5];
            cout << "cube_max_val " << cube_max_val << endl << endl << endl;
        }

        Mat hsvFrame;
        cvtColor(colorFrame, hsvFrame, COLOR_BGR2HSV);
        Mat channels[3];
        split(hsvFrame, channels);

        // Cone filtering
        Mat coneHueMask;
        inRange(channels[0], cone_min_hue, cone_max_hue, coneHueMask);
        Mat coneSatMask;
        inRange(channels[1], cone_min_sat, cone_max_sat, coneSatMask);
        Mat coneValMask;
        inRange(channels[2], cone_min_val, cone_max_val, coneValMask);

        Mat coneMask;
        bitwise_and(coneHueMask, coneSatMask, coneMask);
        bitwise_and(coneMask, coneValMask, coneMask);
        cvtColor(coneMask, coneMask, COLOR_GRAY2BGR);

        Mat justCone(Size(WIDTH, HEIGHT), CV_8UC1);
        bitwise_and(colorFrame, coneMask, justCone);

        // Cube filtering
        Mat cubeHueMask;
        inRange(channels[0], cube_min_hue, cube_max_hue, cubeHueMask);
        Mat cubeSatMask;
        inRange(channels[1], cube_min_sat, cube_max_sat, cubeSatMask);
        Mat cubeValMask;
        inRange(channels[2], cube_min_val, cube_max_val, cubeValMask);

        Mat cubeMask;
        bitwise_and(cubeHueMask, cubeSatMask, cubeMask);
        bitwise_and(cubeMask, cubeValMask, cubeMask);
        cvtColor(cubeMask, cubeMask, COLOR_GRAY2BGR);

        Mat justCube(Size(WIDTH, HEIGHT), CV_8UC1);
        bitwise_and(colorFrame, cubeMask, justCube);

        // Stream all the images
        vector<uchar> buf_colorFrame;
        imencode(".jpg", colorFrame, buf_colorFrame, stream_params);
        streamer.publish("/colorFrame", string(buf_colorFrame.begin(), buf_colorFrame.end()));

        vector<uchar> buf_coneValMask;
        imencode(".jpg", coneValMask, buf_coneValMask, stream_params);
        streamer.publish("/coneValMask", string(buf_coneValMask.begin(), buf_coneValMask.end()));

        vector<uchar> buf_coneHueMask;
        imencode(".jpg", coneHueMask, buf_coneHueMask, stream_params);
        streamer.publish("/coneHueMask", string(buf_coneHueMask.begin(), buf_coneHueMask.end()));

        vector<uchar> buf_coneSatMask;
        imencode(".jpg", coneSatMask, buf_coneSatMask, stream_params);
        streamer.publish("/coneSatMask", string(buf_coneSatMask.begin(), buf_coneSatMask.end()));

        vector<uchar> buf_justCone;
        imencode(".jpg", justCone, buf_justCone, stream_params);
        streamer.publish("/justCone", string(buf_justCone.begin(), buf_justCone.end()));

        vector<uchar> buf_cubeHueMask;
        imencode(".jpg", cubeHueMask, buf_cubeHueMask, stream_params);
        streamer.publish("/cubeHueMask", string(buf_cubeHueMask.begin(), buf_cubeHueMask.end()));

        vector<uchar> buf_cubeSatMask;
        imencode(".jpg", cubeSatMask, buf_cubeSatMask, stream_params);
        streamer.publish("/cubeSatMask", string(buf_cubeSatMask.begin(), buf_cubeSatMask.end()));

        vector<uchar> buf_cubeValMask;
        imencode(".jpg", cubeValMask, buf_cubeValMask, stream_params);
        streamer.publish("/cubeValMask", string(buf_cubeValMask.begin(), buf_cubeValMask.end()));

        vector<uchar> buf_justCube;
        imencode(".jpg", justCube, buf_justCube, stream_params);
        streamer.publish("/justCube", string(buf_justCube.begin(), buf_justCube.end()));

        colorFrame.release();
        hsvFrame.release();
        coneHueMask.release();
        coneValMask.release();
        coneSatMask.release();
        cubeHueMask.release();
        cubeValMask.release();
        cubeSatMask.release();
        justCone.release();
        justCube.release();
    }
}
