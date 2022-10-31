#pragma once

#include <vector>
#include "../parameters.h"
#include "../../Utils/EigenTypes.h"
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <ros/console.h>

using namespace Eigen;
using namespace std;
/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
class InitialEXRotation
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    InitialEXRotation();

    bool
    CalibrationExRotation(const Eigen::aligned_vector<pair<Vector3d, Vector3d>>& corres, const Quaterniond& delta_q_imu, Matrix3d &calib_ric_result);
private:
    Matrix3d solveRelativeR(const Eigen::aligned_vector<pair<Vector3d, Vector3d>> &corres);

    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
    int frame_count;

    Eigen::aligned_vector<Matrix3d> Rc;
    Eigen::aligned_vector<Matrix3d> Rimu;
    Eigen::aligned_vector<Matrix3d> Rc_g;
    Matrix3d ric;
};


