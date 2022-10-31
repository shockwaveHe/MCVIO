#pragma once

#include <vector>
#include <fstream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <ros/ros.h>

#include "../utility/utility.h"
#include "../utility/Twist.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/CameraFactory.h"

// Estimator Parameters start
// -----------------
const double FOCAL_LENGTH = 460.0;

const int WINDOW_SIZE = 10;

extern int NUM_OF_CAM;

const int NUM_OF_F = 1000;

extern camodocal::CameraPtr m_camera;

#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;

extern double MIN_PARALLAX;

extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;

extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;

extern std::vector<Eigen::Vector3d> TIC;

extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;

extern double BIAS_GYR_THRESHOLD;

extern double SOLVER_TIME;

extern int NUM_ITERATIONS;

extern std::string EX_CALIB_RESULT_PATH;

extern std::string VINS_RESULT_PATH;

extern std::string IMU_TOPIC;

extern double TD;

extern double TR;

extern int ESTIMATE_TD;

extern int ROLLING_SHUTTER;

extern double ROW, COL;
// Estimator Parameters start
// -----------------

// FeatureTracking Parameters start
// image, lidar message topic
extern std::string IMAGE_TOPIC;

extern std::string DEPTH_TOPIC;

extern std::string LASER_TOPIC;

extern std::string ODOM_TOPIC;

extern std::string LIO_World_Frame;

extern std::string VINS_World_Frame;

extern std::string Camera_Frame;

extern std::string Laser_Frame;

extern std::string LIO_Laser_Frame;

extern std::string VINS_IMU_Frame;

extern int LASER_TYPE;

// fisheye mask
extern std::string FISHEYE_MASK;

extern std::vector<std::string> CAM_NAMES;

extern int MAX_CNT;

extern int MIN_DIST;

extern int FREQ;

extern double F_THRESHOLD;

extern int SHOW_TRACK;

extern int STEREO_TRACK;

extern int EQUALIZE;

extern int FISHEYE;

extern bool PUB_THIS_FRAME;

extern double m_k1, m_k2, m_p1, m_p2;

extern double fx, fy, cx, cy;

extern double lidar_search_radius;

// debug
extern int estimate_scale;

extern int depth_support_init;

extern double scale;

extern int PnP_simple_init;

extern int skip_default_init_method;

extern Eigen::Matrix3d cam_laser_R;
extern Eigen::Vector3d cam_laser_T;
extern Eigen::Matrix3d imu_laser_R;
extern Eigen::Vector3d imu_laser_T;

extern Transformd Tcam_lidar;
extern Transformd Tlidar_cam;
extern Transformd Timu_lidar;

extern double L_C_TX;
extern double L_C_TY;
extern double L_C_TZ;
extern double L_C_RX;
extern double L_C_RY;
extern double L_C_RZ;
extern int ALIGN_CAMERA_LIDAR_COORDINATE;

extern int SHOW_WINDOW_SIZE;
extern int USE_LIDAR_ODOM_MEAS;
extern int SCALE_LIDAR_ABSOLUTE_FACTOR;

// FeatureTracking Parameters end
//-------------------

// henryzh47: VIO initialzation delay to make sure steady pose sent to LIO
const int VIO_INIT_DELAY = 50;
template <typename T>
T readParam(ros::NodeHandle &n, std::string name);
void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
