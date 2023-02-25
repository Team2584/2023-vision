#include "detection.h"

bool shouldIgnoreDetection(apriltag_detection_t *det, int frame_width, int frame_height)
{
    // Only use valid tag detections
    if (det->id > 8 || det->id < 1)
        return true;

    // Filter so it doesn't use detections close to the edge
    for (int i = 0; i < 4; i++)
    {
        if (in_margin(det->p[i], frame_width, frame_height))
        {
            return true;
        }
    }
    return false;
}

std::vector<robot_position> getPoses(cv::Mat grayFrame, cv::Mat colorFrame, camInfo *cam,
                                     apriltag_detector_t *td)
{

    extern int total_hamm_hist[];
    extern int hamm_hist[];

    // Make an image_u8_t header from the frame
    image_u8_t im = {
        .width = grayFrame.cols,
        .height = grayFrame.rows,
        .stride = grayFrame.cols,
        .buf = grayFrame.data,
    };

    // Detect Tags
    zarray_t *detections = apriltag_detector_detect(td, &im);
    std::vector<robot_position> poses;

    // Filter detections and add good ones to array
    int tags = 0;
    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        if (!shouldIgnoreDetection(det, grayFrame.cols, grayFrame.rows))
        {
            tags++;
            labelDetections(colorFrame, det);

            robot_position pos;
            getRobotPosition(det, &pos, cam);
            poses.push_back(pos);
        }
        hamm_hist[det->hamming]++;
        total_hamm_hist[det->hamming]++;
    }
    apriltag_detections_destroy(detections);
    return poses;
}
