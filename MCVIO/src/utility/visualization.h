#pragma once

#include <Eigen/Dense>
#include <fstream>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "../Frontend/MCVIOfrontend.h"
#include "../Estimator/MCVIOestimator.h"
#include "../Estimator/parameters.h"
#include "CameraPoseVisualization.h"
using namespace MCVIO;
extern ros::Publisher pub_odometry, pub_loam_odometry, pub_latest_odometry;

extern ros::Publisher pub_path, pub_pose;

extern ros::Publisher pub_cloud, pub_map;

extern ros::Publisher pub_key_poses;

extern ros::Publisher pub_ref_pose, pub_cur_pose;

extern ros::Publisher pub_key;

extern nav_msgs::Path path;

extern ros::Publisher pub_pose_graph;

extern int IMAGE_ROW, IMAGE_COL;

void registerPub(ros::NodeHandle &n);

void pubLatestOdometry(const MCVIOEstimator &estimator, const Eigen::Vector3d &P,
                       const Eigen::Quaterniond &Q, const Eigen::Vector3d &V,
                       const std_msgs::Header &header);

void printStatistics(const MCVIOEstimator &estimator, double t);

tf::Transform transformConversion(const tf::StampedTransform& t);

void pubOdometry(const MCVIOEstimator &estimator, const std_msgs::Header &header);

void pubInitialGuess(const MCVIOEstimator &estimator,
                     const std_msgs::Header &header);

void pubKeyPoses(const MCVIOEstimator &estimator, const std_msgs::Header &header);

void pubCameraPose(const MCVIOEstimator &estimator, const std_msgs::Header &header);

void pubSlideWindowPoses(const MCVIOEstimator &estimator,
                         const std_msgs::Header &header);

void pubPointCloud(const MCVIOEstimator &estimator, const std_msgs::Header &header);

// void pubWindowLidarPointCloud(const MCVIOEstimator &estimator,
//                               const std_msgs::Header &header);

void pubTF(const MCVIOEstimator &estimator, const std_msgs::Header &header);

void pubKeyframe(const MCVIOEstimator &estimator);

void pubRelocalization(const MCVIOEstimator &estimator);
