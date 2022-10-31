#ifndef MCVIOFRONTEND_DATA_H
#define MCVIOFRONTEND_DATA_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../Utils/EigenTypes.h"
#include "../utility/Twist.h"
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
namespace MCVIO
{
    using FeatureID = int;
    using TimeFrameId = double;
    //
    // previous: id: [cam, feature]
    // current:  id: feature
    using FeatureTrackerResults =
        Eigen::aligned_map<int,
                           Eigen::aligned_vector<Eigen::Matrix<double, 8, 1>>>;

    struct FrontEndResult
    {
        typedef std::shared_ptr<FrontEndResult> Ptr;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // optical flow result
        // std::vector<double> timestamps;
        std::vector<FeatureTrackerResults> features;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
        std::vector<cv::Mat> images;

        // laser odometry measurement
        Transformd Tw_imu_meas;
        Eigen::Vector3d vel_imu_meas;
        Eigen::Vector3d Ba_meas;
        Eigen::Vector3d Bg_meas;
        double gravity_meas;
        int reset_id; // to notifiy the status of laser odometry
        bool laser_odom_vio_sync = false;
    };
} // namespce MCVIO

#endif // FRONTEND_DATA_H
