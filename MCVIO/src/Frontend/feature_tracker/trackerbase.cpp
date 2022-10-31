#include "trackerbase.h"
using namespace MCVIO;

TrackerBase::TrackerBase()
{
    n_id = 0;
}

TrackerBase::~TrackerBase()
{
}

void TrackerBase::readImage(const cv::Mat &_img, double _cur_time)
{
    LOG(INFO) << "TrackerBase does not implement function: " << __func__;
};

void TrackerBase::setMask()
{
    LOG(INFO) << "TrackerBase does not implement function: " << __func__;
};

void TrackerBase::addPoints()
{
    LOG(INFO) << "TrackerBase does not implement function: " << __func__;
};

bool TrackerBase::updateID(unsigned int i)
{
    LOG(INFO) << "TrackerBase does not implement function: " << __func__;
    return true;
};

void TrackerBase::readIntrinsicParameter(const string &calib_file)
{
    LOG(INFO) << "TrackerBase does not implement function: " << __func__;
};

void TrackerBase::showUndistortion(const string &name)
{
    LOG(INFO) << "TrackerBase does not implement function: " << __func__;
};

void TrackerBase::rejectWithF()
{
    LOG(INFO) << "TrackerBase does not implement function: " << __func__;
};

void TrackerBase::undistortedPoints()
{
    LOG(INFO) << "TrackerBase does not implement function: " << __func__;
};

void TrackerBase::getPt(int idx, int &id, geometry_msgs::Point32 &p, cv::Point2f &p_uv, cv::Point2f &v)
{
    LOG(INFO) << "TrackerBase does not implement function: " << __func__;
};
void TrackerBase::getCurPt(int idx, cv::Point2f &cur_pt)
{
    LOG(INFO) << "TrackerBase does not implement function: " << __func__;
};

void TrackerBase::Lock(){
    /* Lock VPI array */
};
void TrackerBase::Unlock(){
    /* Unlock VPI array */
};

void TrackerBase::reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void TrackerBase::reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
