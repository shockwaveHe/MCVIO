#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "../../Utils/EigenTypes.h"
#include "../parameters.h"
using namespace Eigen;
using namespace std;

#define USE_DEPTH_INIT 0
struct SFMFeature
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool state; 
    int id;
    double position[3];
    double depth;
    vector<pair<int, double>> observation_depth;           
    Eigen::aligned_vector<pair<int, Vector2d>> observation; 
};

struct ReprojectionError3D
{
    ReprojectionError3D(double observed_u, double observed_v)
        : observed_u(observed_u), observed_v(observed_v)
    {
    }

    template <typename T>
    bool operator()(const T *const camera_R, const T *const camera_T, const T *point, T *residuals) const
    {
        T p[3];
        ceres::QuaternionRotatePoint(camera_R, point, p);
        p[0] += camera_T[0];
        p[1] += camera_T[1];
        p[2] += camera_T[2];
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];
        residuals[0] = xp - T(observed_u);
        residuals[1] = yp - T(observed_v);
        return true;
    }

    static ceres::CostFunction *Create(const double observed_x,
                                       const double observed_y)
    {
        return (new ceres::AutoDiffCostFunction<
                ReprojectionError3D, 2, 4, 3, 3>(
            new ReprojectionError3D(observed_x, observed_y)));
    }

    double observed_u;
    double observed_v;
};

class GlobalSFM
{
public:
    GlobalSFM();
    bool construct(int frame_num,
                   Eigen::aligned_vector<Eigen::aligned_vector<Quaterniond>> &q,
                   Eigen::aligned_vector<Eigen::aligned_vector<Vector3d>> &T,
                   int l,
                   int base_cam,
                   const Matrix3d relative_R,
                   const Vector3d relative_T,
                   Eigen::aligned_vector<Eigen::aligned_vector<SFMFeature>> &sfm_f,
                   Eigen::aligned_vector<Eigen::aligned_map<int, Vector3d>> &sfm_tracked_points,
                   Matrix3d ric[],
                   Vector3d tic[]);

private:
    bool
    solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial,
                    int i, Eigen::aligned_vector<Eigen::aligned_vector<SFMFeature>> &sfm_f, int base_cam);

    void
    triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                     Eigen::Matrix<double, 3, 4> &Pose1,
                     Vector2d &point0,
                     Vector2d &point1,
                     Vector3d &point_3d);

    void triangulateTwoFrames(
        int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
        int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
        Eigen::aligned_vector<Eigen::aligned_vector<SFMFeature>> &sfm_f, int cam);

    void
    triangulateTwoFramesWithDepth(int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
                                  int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
                                  Eigen::aligned_vector<Eigen::aligned_vector<SFMFeature>> &sfm_f, int cam);

    vector<int> feature_num;
};