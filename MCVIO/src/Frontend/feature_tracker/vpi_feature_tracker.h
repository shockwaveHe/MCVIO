#pragma once
// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION >= 3
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#else
#include <opencv2/highgui/highgui.hpp>
#endif

#include <opencv2/imgproc/imgproc.hpp>
#include <vpi/OpenCVInterop.hpp>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <numeric>
#include <sstream>
#include <vector>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
// VPI
#include <vpi/Array.h>
#include <vpi/Image.h>
#include <vpi/Pyramid.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/GaussianPyramid.h>
#include <vpi/algo/HarrisCorners.h>
#include <vpi/algo/OpticalFlowPyrLK.h>
#include <vpi/Version.h>

#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "trackerbase.h"
#include "../sensors.h"

#define USE_GPU 0
#define quadTreePickStrategy 1 // 1 means pick feature points according to scores. 0 means uniformly select feature points
using namespace std;
using namespace camodocal;
using namespace Eigen;
using namespace MCVIO;

constexpr int MAX_HARRIS_CORNERS = 1024;
constexpr int MAX_KEYPOINTS = 200;
constexpr int MIN_KEYPOINTS = 150;
constexpr int BORDER_SIZE = 1;
struct node
{
    int beginIdx;
    int endIdx;
    float minRow;
    float minCol;
    float maxRow;
    float maxCol;
};

// This is used to debugging VPI functions, when finish debugging, remember to move them to reduce latency
#define CHECK_STATUS(STMT)                                    \
    do                                                        \
    {                                                         \
        VPIStatus status = (STMT);                            \
        if (status != VPI_SUCCESS)                            \
        {                                                     \
            char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];       \
            vpiGetLastStatusMessage(buffer, sizeof(buffer));  \
            std::ostringstream ss;                            \
            ss << vpiStatusGetName(status) << ": " << buffer; \
            throw std::runtime_error(ss.str());               \
        }                                                     \
    } while (0);

void outputVPIKeypoints(VPIArray src);
// void outputVPIStatus(VPIArray src);
// copy the data stored in cv point vector to VPI keypoint array
void cvPointVec_to_VPIKeyPointArr(vector<cv::Point2f> &src, VPIArray &dst);
// copy the data stored in VPI keypoint array to cv point vector
void VPIKeyPointArr_to_cvPointVec(VPIArray &src, vector<cv::Point2f> &dst);
void VPIstatus_to_cvStatus(VPIArray &src, vector<uchar> &dst);
void SortKeypoints(VPIArray keypoints, VPIArray scores, std::size_t max);

vector<node> splitNode(vector<cv::Point2f> &v, vector<float> &scores, vector<node> &info);

class VPIFeatureTracker : public TrackerBase
{
public:
    VPIFeatureTracker();

    void readImage(const cv::Mat &_img, double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    bool inBorder(const cv::Point2f &pt);
    bool inBorder(const VPIKeypoint &pt);

    void initVPIData(const sensor_msgs::ImageConstPtr &img_msg);

    void reduceVectorVPI(VPIArray v, VPIArray arrStatus);
    void reduceVectorVPI(VPIArray v, vector<uchar> &arrStatus);
    void reduceVectorVPI(vector<int> &v, VPIArray status);
    void reduceVectorVPI2(vector<cv::Point2f> &v, VPIArray status);
    void reduceVector(vector<cv::Point2f> &v, vector<uchar> &status, VPIArray &correspondingVPIarr);
    void reduceVector(vector<int> &v, vector<uchar> &status, VPIArray &correspondingVPIarr);
    void reduceVector1(vector<cv::Point2f> &v, vector<uchar> status, VPIArray correspondingVPIarr = NULL);

    void getPt(int idx, int &id, geometry_msgs::Point32 &p, cv::Point2f &p_uv, cv::Point2f &v);
    void getCurPt(int idx, cv::Point2f &cur_pt);

    void Lock();
    void Unlock();

    vector<cv::Point2f> pickPtsByQuatTree(VPIArray src, VPIArray scores, std::size_t amount);
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> pts_velocity;
    // vector<int> ids;
    // vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    // camodocal::CameraPtr m_camera;
    double cur_time;
    double prev_time;
    // static int n_id;
    // VPI data
    VPIStream stream;
    VPIImage VPI_prev_img, VPI_cur_img, VPI_forw_img;
    VPIPyramid pyrCurFrame, pyrForwFrame;
    VPIArray arrPrevPts, arrCurPts, arrForwPts, arrStatus;
    VPIArray VPI_n_pts;
    VPIPayload optflow;
    VPIArray scores;
    VPIPayload harris;
    VPIBackend backend;
    // cv::Mat cvPrevFrame;   // correspond cur
    // cv::Mat cvCurFrame;     //correspond to forw
    VPIImageFormat imgFormat;
    VPIOpticalFlowPyrLKParams lkParams;
    VPIHarrisCornerDetectorParams harrisParams;
    int32_t pyrLevel;
    int round;

    std::shared_ptr<MCVIO::MCVIOcamera> cam;

    VPIArrayData cur_data;
    VPIKeypoint* cur_data_pts;
};
