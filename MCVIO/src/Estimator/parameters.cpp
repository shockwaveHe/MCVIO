#include "parameters.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

camodocal::CameraPtr m_camera;

double INIT_DEPTH;

double MIN_PARALLAX;

double ACC_N, ACC_W;

double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;

std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;

double BIAS_GYR_THRESHOLD;

double SOLVER_TIME;

int NUM_ITERATIONS;

int NUM_OF_CAM;

int ESTIMATE_EXTRINSIC;

int ESTIMATE_TD;

int ROLLING_SHUTTER;

std::string EX_CALIB_RESULT_PATH;

std::string VINS_RESULT_PATH;

std::string IMU_TOPIC;

double ROW, COL;

double TD, TR;

double L_C_TX;
double L_C_TY;
double L_C_TZ;
double L_C_RX;
double L_C_RY;
double L_C_RZ;

int ALIGN_CAMERA_LIDAR_COORDINATE;

// FeatureTracking Parameters start
// image, lidar message topic
std::string IMAGE_TOPIC;

std::string DEPTH_TOPIC;

std::string LASER_TOPIC;

std::string ODOM_TOPIC;

std::string LIO_World_Frame;

std::string VINS_World_Frame;

std::string Camera_Frame;

std::string Laser_Frame;

std::string LIO_Laser_Frame;

std::string VINS_IMU_Frame;

int LASER_TYPE;

// fisheye mask
std::string FISHEYE_MASK;

std::vector<std::string> CAM_NAMES;

int MAX_CNT;

int MIN_DIST;

int FREQ;

double F_THRESHOLD;

int SHOW_TRACK;

int STEREO_TRACK;

int EQUALIZE;

int FISHEYE;

bool PUB_THIS_FRAME;

double m_k1, m_k2, m_p1, m_p2;

double fx, fy, cx, cy;

double lidar_search_radius;

int estimate_scale;

int depth_support_init;

int skip_default_init_method;

double scale;

int PnP_simple_init;

Eigen::Matrix3d cam_laser_R;

Eigen::Vector3d cam_laser_T;

Transformd Tcam_lidar;

Transformd Timu_lidar;

Eigen::Matrix3d imu_laser_R;

Eigen::Vector3d imu_laser_T;

int SHOW_WINDOW_SIZE;
int USE_LIDAR_ODOM_MEAS;
int SCALE_LIDAR_ABSOLUTE_FACTOR;
// FeatureTracking Parameters end
//-------------------

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        LOG(INFO) << "Loaded " << name << ": " << ans;
    }
    else
    {
        LOG(ERROR) << "Failed to load " << name;
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    LOG(INFO) << "[vins estimator] read parameter";
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    LOG(INFO) << "[vins estimator] config_file: " << config_file;
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // read FeatureTracking config start
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");
    LOG(INFO) << "\nvins_folder" << VINS_FOLDER_PATH;
    // fsSettings["image_topic"] >> IMAGE_TOPIC;
    // fsSettings["depth_topic"] >> DEPTH_TOPIC;
    // fsSettings["laser_topic"] >> LASER_TOPIC;
    // fsSettings["odom_topic"] >> ODOM_TOPIC;

    LIO_World_Frame = readParam<std::string>(n, "lio_world_frame");
    VINS_World_Frame = readParam<std::string>(n, "vins_world_frame");
    // Camera_Frame = readParam<std::string>(n, "camera_frame");
    Laser_Frame = readParam<std::string>(n, "laser_frame");
    LIO_Laser_Frame = readParam<std::string>(n, "lio_laser_frame");
    VINS_IMU_Frame = readParam<std::string>(n, "vins_imu_frame");

    if (!n.getParam("lio_world_frame", LIO_World_Frame))
        LIO_World_Frame = "sensor_init";
    if (!n.getParam("vins_world_frame", VINS_World_Frame))
        VINS_World_Frame = "vins_world";
    if (!n.getParam("camera_frame", Camera_Frame))
        Camera_Frame = "camera";
    if (!n.getParam("laser_frame", Laser_Frame))
        Laser_Frame = "sensor";
    if (!n.getParam("lio_laser_frame", LIO_Laser_Frame))
        LIO_Laser_Frame = "sensor";
    if (!n.getParam("vins_imu_frame", VINS_IMU_Frame))
        VINS_IMU_Frame = "vins_imu";

    // L_C_TX = fsSettings["lidar_to_cam_tx"];
    // L_C_TY = fsSettings["lidar_to_cam_ty"];
    // L_C_TZ = fsSettings["lidar_to_cam_tz"];
    // L_C_RX = fsSettings["lidar_to_cam_rx"];
    // L_C_RY = fsSettings["lidar_to_cam_ry"];
    // L_C_RZ = fsSettings["lidar_to_cam_rz"];

    fsSettings["align_camera_lidar_estimation"] >> ALIGN_CAMERA_LIDAR_COORDINATE;

    // LASER_TYPE = fsSettings["laser_type"];
    USE_LIDAR_ODOM_MEAS = fsSettings["use_lidar_odom_meas"];
    LOG(INFO) << "\nuse lidar odom meas: " << USE_LIDAR_ODOM_MEAS;

    SCALE_LIDAR_ABSOLUTE_FACTOR = fsSettings["scale_lidar_absolute_factor"];
    LOG(INFO) << "\n scale lidar absolute factor: " << SCALE_LIDAR_ABSOLUTE_FACTOR;

    // LOG(INFO) << "\nimage_topic: " << IMAGE_TOPIC
            //   << "\ndepth_topic: " << DEPTH_TOPIC
            //   << "\nlaser_topic: " << LASER_TOPIC
            //   << "\nodom_topic: " << ODOM_TOPIC;

    // LOG(INFO) << "\nlaser_type: " << LASER_TYPE;

    // SHOW_WINDOW_SIZE = 20;
    // STEREO_TRACK = false;
    // PUB_THIS_FRAME = false;

    // if (FREQ == 0)
    //     FREQ = 100;

    // read FeatureTracking config end

    fsSettings["imu_topic"] >> IMU_TOPIC;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    
    lidar_search_radius = fsSettings["lidar_search_radius"];
    estimate_scale = fsSettings["estimate_scale"];
    depth_support_init = 0;
    scale = fsSettings["scale"];
    PnP_simple_init = fsSettings["PnP_simple_init"];
    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    skip_default_init_method = fsSettings["skip_default_init_method"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        // RIC.push_back(Eigen::Matrix3d::Identity());
        // TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
    }
    else
    {
        if (ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");
    }


    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }

    fsSettings.release();
}
