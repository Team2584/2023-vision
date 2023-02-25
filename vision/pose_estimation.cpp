#include "pose_estimation.h"
#include "Cameras.h"
#include "main.h"

using namespace std;

//----------- From AprilNav -----------------------------
// Normalize angle to be within the interval [-pi,pi].
inline double standardRad(double t)
{
    if (t >= 0.)
    {
        t = fmod(t + M_PI, M_TWOPI) - M_PI;
    }
    else
    {
        t = fmod(t - M_PI, -M_TWOPI) + M_PI;
    }
    return t;
}

tag_position get_tag_position(int id)
{
    tag_position pos;
    double field_len = 54 * 12 + 3.25;

    double blue_grid_y = 3 * 12 + 4.25;
    double red_grid_y = field_len - blue_grid_y;

    double blue_sub_y = 14;
    double red_sub_y = field_len - blue_sub_y;

    double grid_z = 12 + 6.25;
    double sub_z = 2 * 12 + 3.375;

    switch (id)
    {
    case 1:
        pos.x = 20 * 12 + 4.5;
        pos.y = red_grid_y;
        pos.z = grid_z;
        pos.theta = 0;
        break;
    case 2:
        pos.x = 17 * 12 + 3;
        pos.y = red_grid_y;
        pos.z = grid_z;
        pos.theta = 0;
        break;
    case 7:
        pos.x = 14 * 12 + 11;
        pos.y = blue_grid_y;
        pos.z = grid_z;
        pos.theta = 0;
        break;

    case 4:
        pos.x = 49.5;
        pos.y = red_sub_y;
        pos.z = sub_z;
        pos.theta = 0;
        break;

    case 5:
        pos.x = 49.5;
        pos.y = blue_sub_y;
        pos.z = sub_z;
        pos.theta = M_PI;
        break;

    case 6:
        pos.x = 11 * 12 + 4.5;
        pos.y = blue_grid_y;
        pos.z = grid_z;
        pos.theta = M_PI;
        break;
    case 3:
        pos.x = 17 * 12 + 3;
        pos.y = blue_grid_y;
        pos.z = grid_z;
        pos.theta = M_PI;
        break;
    case 8:
        pos.x = 20 * 12 + 4.5;
        pos.y = blue_grid_y;
        pos.z = grid_z;
        pos.theta = M_PI;
        break;
    }
    pos.x /= INCH;
    pos.y /= INCH;
    pos.z /= INCH;
    return pos;
}

// Convert rotation matrix to Euler angles
void wRo_to_euler(const Eigen::Matrix3d &wRo, double &yaw, double &pitch, double &roll)
{
    yaw = standardRad(atan2(wRo(1, 0), wRo(0, 0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2, 0), wRo(0, 0) * c + wRo(1, 0) * s));
    roll = standardRad(atan2(wRo(0, 2) * s - wRo(1, 2) * c, -wRo(0, 1) * s + wRo(1, 1) * c));
}

Eigen::Matrix4d getRelativeTransform(apriltag_detection_t *det, camInfo *cam)
{
    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> imgPts;
    double s = TAG_SIZE / 2.;
    objPts.push_back(cv::Point3f(-s, -s, 0));
    objPts.push_back(cv::Point3f(s, -s, 0));
    objPts.push_back(cv::Point3f(s, s, 0));
    objPts.push_back(cv::Point3f(-s, s, 0));

    std::pair<float, float> p1(det->p[0][0], det->p[0][1]);
    std::pair<float, float> p2(det->p[1][0], det->p[1][1]);
    std::pair<float, float> p3(det->p[2][0], det->p[2][1]);
    std::pair<float, float> p4(det->p[3][0], det->p[3][1]);
    imgPts.push_back(cv::Point2f(p1.first, p1.second));
    imgPts.push_back(cv::Point2f(p2.first, p2.second));
    imgPts.push_back(cv::Point2f(p3.first, p3.second));
    imgPts.push_back(cv::Point2f(p4.first, p4.second));

    cv::Mat rvec, tvec;

    cv::solvePnP(objPts, imgPts, cam->camMatx, cam->distCoeffs, rvec, tvec);
    cv::Matx33d r;
    cv::Rodrigues(rvec, r);
    Eigen::Matrix3d wRo;
    wRo << r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2);

    Eigen::Matrix4d T;
    T.topLeftCorner(3, 3) = wRo;
    T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    T.row(3) << 0, 0, 0, 1;

    return T;
}

void getRelativeTranslationRotation(apriltag_detection_t *det, camInfo *cam, Eigen::Vector3d &trans,
                                    Eigen::Matrix3d &rot)
{
    Eigen::Matrix4d T = getRelativeTransform(det, cam);

    // converting from camera frame (z forward, x right, y down) to
    // object frame (x forward, y left, z up)
    Eigen::Matrix4d M;
    M << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix4d MT = M * T;
    // translation vector from camera to the April tag
    trans = MT.col(3).head(3);
    // orientation of April tag with respect to camera: the camera
    // convention makes more sense here, because yaw,pitch,roll then
    // naturally agree with the orientation of the object
    rot = T.block(0, 0, 3, 3);
    /*
     * Positive X = Camera moving left
     * Positive Z = Camera moving up
     * Positive Y = Camera moving away from tag
     */
}

// ------------------ NOT from AprilNav ---------------------------------
Eigen::Vector3d getRealTranslationRotation(Eigen::Vector3d pt, Eigen::Matrix3d rot)
{
    // cout << "Point:" << pt << endl;
    // cout << "Rotation Matrix:\n" << rot << endl;
    Eigen::Vector3d newpt = -rot.transpose() * pt;
    return newpt;
}

Eigen::Matrix3d rotation_from_euler(double roll, double pitch, double yaw)
{
    // roll and pitch and yaw in radians
    double su = sin(roll);
    double cu = cos(roll);
    double sv = sin(pitch);
    double cv = cos(pitch);
    double sw = sin(yaw);
    double cw = cos(yaw);
    Eigen::Matrix3d Rot_matrix(3, 3);
    Rot_matrix(0, 0) = cv * cw;
    Rot_matrix(0, 1) = su * sv * cw - cu * sw;
    Rot_matrix(0, 2) = su * sw + cu * sv * cw;
    Rot_matrix(1, 0) = cv * sw;
    Rot_matrix(1, 1) = cu * cw + su * sv * sw;
    Rot_matrix(1, 2) = cu * sv * sw - su * cw;
    Rot_matrix(2, 0) = -sv;
    Rot_matrix(2, 1) = su * cv;
    Rot_matrix(2, 2) = cu * cv;
    return Rot_matrix;
}

int sgn(double x)
{
    if (x > 0)
    {
        return 1;
    }
    else if (x < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

void getRobotPosition(apriltag_detection_t *det, robot_position *pos, camInfo *cam)
{
    Eigen::Vector3d tag_trans;
    Eigen::Matrix3d tag_rot;
    double rotX;
    double rotY;
    double rotZ;

    getRelativeTranslationRotation(det, cam, tag_trans, tag_rot);
    wRo_to_euler(tag_rot, rotY, rotZ, rotX);
    rotX = sgn(rotX) * (M_PI - fabs(rotX));
    rotY *= 1;
    rotZ *= -1;
    // Eigen::Matrix3d rotationMatrix = rotation_from_euler(rotY, rotX, rotZ);
    cout << "Original numbers out of getRelativeTranslationRotation:\n " << tag_trans * INCH
         << endl;
    // cout << "Rotation\n " << tag_rot << endl;
    // printf("Better Rotation\n %f\n %f\n %f\n", rotX, rotY, rotZ);

    // printf("Pre Rotated: \n x: %f\n h: %f\n z: %f\n", linX, linY, linZ);

    // Put the vector in x, y, z in our preferred coordinate system
    Eigen::Vector3d newTagTrans;
    newTagTrans << tag_trans(1), tag_trans(0), tag_trans(2);

    cout << "New tagTagTrans (should be same as above with some swapped numbers):\n"
         << newTagTrans << endl;

    // Rotate from camera frame -> robot frame
    /*
    Eigen::Matrix3d zCamRotMatx;
    zCamRotMatx << 0, 0, 0, 0, 0, 0, 0, 0, 1;
    zCamRotMatx(1, 0) = cos(cam->offset.theta);
    zCamRotMatx(0, 1) = -sin(cam->offset.theta);
    zCamRotMatx(1, 0) = sin(cam->offset.theta);
    zCamRotMatx(1, 1) = cos(cam->offset.theta);
    */

    Eigen::Matrix3d xCamRotMatx;
    xCamRotMatx << 1, 0, 0, 0, 0, 0, 0, 0, 0;
    xCamRotMatx(1, 1) = cos(cam->offset.elevAngle);
    xCamRotMatx(1, 2) = sin(cam->offset.elevAngle);
    xCamRotMatx(2, 1) = -sin(cam->offset.elevAngle);
    xCamRotMatx(2, 2) = cos(cam->offset.elevAngle);

    // newTagTrans = zCamRotMatx * newTagTrans;
    newTagTrans = xCamRotMatx * newTagTrans;

    cout << "newTagTrans after rotating by camera relative to robot:\n" << newTagTrans << endl;

    // Translate from camera to center of robot
    // TODO might be different on different alliance
    newTagTrans(0) -= cam->offset.x;
    newTagTrans(1) += cam->offset.y;
    newTagTrans(2) += cam->offset.z;

    cout << "newTagTrans after translating by camera relative to robot:\n" << newTagTrans << endl;

    // newTagTrans << tag_trans(1) + cam->offset.x, tag_trans(0) + cam->offset.y,
    // tag_trans(2) - cam->offset.z;

    /*
     * Camera moves right -> X grows
     * Camera moves away from target -> Y grows
     * Camera moves down -> Z grows
     */

    /*
     * Camera Rotates clockwise (top view) -> rotZ decreases
     * Camera Rotates clockwise (back view) -> rotY decreases
     * Camera Rotates clockwise (look from camera's right [usb port side]) (i.e. tilts down) -> rotX
     * decreases
     */

    Eigen::Matrix3d zRotMatx;
    zRotMatx << 0, 0, 0, 0, 0, 0, 0, 0, 1;
    zRotMatx(0, 0) = cos(rotZ);
    zRotMatx(0, 1) = -sin(rotZ);
    zRotMatx(1, 0) = sin(rotZ);
    zRotMatx(1, 1) = cos(rotZ);

    /*
    Eigen::Matrix3d xRotMatx;
    xRotMatx << 1, 0, 0, 0, 0, 0, 0, 0, 0;
    xRotMatx(1, 1) = cos(rotX);
    xRotMatx(1, 2) = -sin(rotX);
    xRotMatx(2, 1) = sin(rotX);
    xRotMatx(2, 2) = cos(rotX);
    */

    /*
    Eigen::Matrix3d yRotMatx;
    yRotMatx << 0, 0, 0, 0, 1, 0, 0, 0, 0;
    yRotMatx(0, 0) = cos(rotY);
    yRotMatx(0, 2) = sin(rotY);
    yRotMatx(2, 0) = -sin(rotY);
    yRotMatx(2, 2) = cos(rotY);
    */

    Eigen::Matrix3d fullRotMatx = zRotMatx; //* yRotMatx * xRotMatx;

    Eigen::Vector3d point = fullRotMatx * newTagTrans;

    cout << "position on field (rotated by tag relative to robot):\n" << point << endl;

    // Eigen::Vector3d point = getRealTranslationRotation(tag_trans, rotationMatrix);

    // printf(" x: %f\n y: %f\n z: %f\n", point(0) * INCH, point(1) * INCH, point(2) * INCH);
    printf(" rotX: %f\n rotY: %f\n rotZ: %f\n", rotX / M_PI * 180, rotY / M_PI * 180,
           rotZ / M_PI * 180);

    /*
    pos->x = 4.42 - point(0);
    pos->y = point(1) + 1;
    pos->z = 0.514 - point(2);
    */

    tag_position tag_pos = get_tag_position(det->id);
    cout << "Tag id: " << det->id << endl;
    printf("Tag position:\n x = %f\n y = %f\n z = %f\n", tag_pos.x, tag_pos.y, tag_pos.z);

    // TODO The signs have to be opposite when looking at tag on other side
    double x = -point(0);
    double y = point(1);
    double z = -point(2);

    if (det->id == 4 || det->id == 5)
    {
        x *= -1;
        y *= -1;
    }

    cout << "Final values:" << endl;
    printf("X: %f\nY: %f\nZ:%f\n", x, y, z);

    pos->x = x;
    pos->y = y;
    pos->z = z;
    pos->theta = -1 * (rotZ + M_PI);
}
