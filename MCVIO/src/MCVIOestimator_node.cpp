
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>

#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>

#include "Estimator/MCVIOestimator.h"
#include "Frontend/MCVIOfrontend.h"
#include "Estimator/parameters.h"
#include "utility/visualization.h"
#include "utility/MapRingBuffer.h"

using namespace std;
using namespace MCVIO;
MCVIOEstimator estimator;

std::condition_variable con;

double current_time = -1;

queue<sensor_msgs::ImuConstPtr> imu_buf;

queue<sensor_msgs::PointCloudConstPtr> feature_buf;

queue<sensor_msgs::PointCloudConstPtr> relo_buf;

int sum_of_wait = 0;

std::mutex m_buf;

std::mutex m_state;

std::mutex m_estimator;

double latest_time;

Eigen::Vector3d tmp_P;

Eigen::Quaterniond tmp_Q;

Eigen::Vector3d tmp_V;

Eigen::Vector3d tmp_Ba;

Eigen::Vector3d tmp_Bg;

Eigen::Vector3d acc_0;

Eigen::Vector3d gyr_0;

bool init_feature = 0;

bool init_imu = 1;

double last_imu_t = 0;

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    //    std_msgs::Header header = imu_msg->header;
    //    header.frame_id = "world";
    //    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    //        pubLatestOdometry(estimator, tmp_P, tmp_Q, tmp_V, header);

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());
}

MCVIO::imuAndFrontEndMeasurement
getImuAndFrontEndMeasurements()
{
#if SHOW_LOG_DEBUG
    LOG(INFO) << "Get measurements";
#endif
    bool laser_vio_sync = false;
    MCVIO::FrontEndResultsSynchronizer *synchronizer = &estimator.frontend_->synchronizer;
    MCVIO::imuAndFrontEndMeasurement measurements;

    while (true)
    {
        if (synchronizer->Sync())
        {
#if SHOW_LOG_DEBUG
            LOG(INFO)
                << "Sync success";
            LOG(INFO) << "Stamp:" << synchronizer->sync_results.front()->sync_timestamp;
#endif
        }
        if (imu_buf.empty() || synchronizer->sync_results.empty())
            return measurements;

        if (imu_buf.back()->header.stamp.toSec() <= synchronizer->sync_results.front()->sync_timestamp + estimator.td)
        {
            ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (imu_buf.front()->header.stamp.toSec() >= synchronizer->sync_results.front()->sync_timestamp + estimator.td)
        {
            ROS_WARN("throw img, only should happen at the beginning");
            synchronizer->sync_results.pop();
            continue;
        }
#if SHOW_LOG_DEBUG
        LOG(INFO) << "Construct measurements";
#endif
        auto frontend_msg = synchronizer->sync_results.front();
#if SHOW_LOG_DEBUG
        LOG(INFO) << "Result size:" << frontend_msg->results.size();
#endif
        synchronizer->sync_results.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < frontend_msg->sync_timestamp + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
            if (imu_buf.empty())
            {
#if SHOW_LOG_DEBUG
                LOG(INFO) << "IMU buffer empty!";
#endif
            }
        }

        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, frontend_msg);
    }

    return measurements;
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder! %f", imu_msg->header.stamp.toSec());
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

#if 1
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
#else
    concurrent_imu_queue.push(imu_msg);
#endif
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        // predict imu (no residual error)
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == MCVIOEstimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(estimator, tmp_P, tmp_Q, tmp_V, header);
    }
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
#if 1
        m_buf.lock();
        while (!feature_buf.empty())
            feature_buf.pop();
        while (!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
#else
        sensor_msgs::ImuConstPtr imuMsgPtr;
        while (!concurrent_imu_queue.empty())
            concurrent_imu_queue.pop(imuMsgPtr);

        sensor_msgs::PointCloudConstPtr featurePtr;
        while (!concurrent_feature_queue.empty())
            concurrent_feature_queue.pop(featurePtr);
#endif
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    // printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

// thread: visual-inertial odometry

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
void process1()
{
    while (true)
    {
        MCVIO::imuAndFrontEndMeasurement measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 { return !(measurements = getImuAndFrontEndMeasurements()).empty(); });
        lk.unlock();
        m_estimator.lock();
#if SHOW_LOG_DEBUG
        LOG(INFO) << "Process IMU and feature msgs";
#endif
        for (auto &measurement : measurements)
        {
            auto &frontend_msg = measurement.second;
            auto &feature_msg = frontend_msg->results;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            double img_t = frontend_msg->sync_timestamp + estimator.td;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();

                if (t <= img_t)
                {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
#if SHOW_LOG_DEBUG
            LOG(INFO) << "Finish process IMU";
#endif
            // set relocalization frame
            sensor_msgs::PointCloudConstPtr relo_msg = nullptr;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != nullptr)
            {
                Eigen::aligned_vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (const auto &point : relo_msg->points)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = point.x;
                    u_v_id.y() = point.y;
                    u_v_id.z() = point.z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3],
                                   relo_msg->channels[0].values[4],
                                   relo_msg->channels[0].values[5],
                                   relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                int c_cur = relo_msg->channels[0].values[8];
                int c_old = relo_msg->channels[0].values[9];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r, c_cur, c_old);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", frontend_msg->sync_timestamp);

            TicToc t_s;

            std_msgs::Header header;
            header.stamp = ros::Time().fromSec(frontend_msg->sync_timestamp);
#if SHOW_LOG_DEBUG
            LOG(INFO) << "Process image and lidar";
            LOG(INFO) << "Result size:" << frontend_msg->results.size();
#endif
            estimator.processImageAndLidar(*frontend_msg);

            double whole_t = t_s.toc();

            printStatistics(estimator, whole_t);

            header.frame_id = VINS_World_Frame;
            // utility/visualization.cpp

            pubOdometry(estimator, header);
            pubSlideWindowPoses(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubKeyframe(estimator);
            // if (relo_msg != nullptr)
            //     pubRelocalization(estimator);
            // ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == MCVIOEstimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}
#pragma clang diagnostic pop

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MCVIO_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    string config_file = readParam<string>(n, "config_file");

    LOG(INFO) << "Config_file:" << config_file;

    MCVIOfrontend frontend(config_file);
    MCVIOfrontend_ = &frontend;
    frontend.setUpROS(nullptr, &n);
    // LOAD TIC RIC
    NUM_OF_CAM = (int)frontend.tracker_tag.size();
    LOG(INFO) << "NUM_OF_CAM:" << NUM_OF_CAM;
    TIC.resize(NUM_OF_CAM);
    RIC.resize(NUM_OF_CAM);
    TIC.clear();
    RIC.clear();
    std::vector<std::string> names;
    names.resize(NUM_OF_CAM);
    for (auto i : frontend.tracker_tag)
    {
        std::string name = i.first;
        int idx = i.second;

        auto sensor = frontend.sensors[frontend.sensors_tag[name]];
        TIC[idx] = sensor->ext_T;
        RIC[idx] = sensor->ext_R;
        names[idx] = name;
    }
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        LOG(INFO) << names[c];
        LOG(INFO) << "EXT_R: \n"
                  << RIC[c];
        LOG(INFO) << "EXT_T: \n"
                  << TIC[c];
    }
    estimator.init(&frontend);
    estimator.setParameter();
    // initial frontend end
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");
    LOG(INFO) << "Register publishers";
    registerPub(n);
    LOG(INFO) << "Finish initialization";
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());

    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);

    std::thread measurement_process{process1};

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
