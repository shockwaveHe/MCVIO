#include "vpi_feature_tracker.h"
using namespace MCVIO;
// int FeatureTracker::n_id = 0;
int filecount = 0;

bool VPIFeatureTracker::inBorder(const cv::Point2f &pt)
{
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < cam->COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < cam->ROW - BORDER_SIZE;
}
bool VPIFeatureTracker::inBorder(const VPIKeypoint &pt)
{
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < cam->COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < cam->ROW - BORDER_SIZE;
}

void VPIFeatureTracker::reduceVectorVPI(VPIArray v, VPIArray arrStatus)
{
    int j = 0;
    VPIArrayData arrData, statusdata;
    VPIKeypoint *arrPoints;
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(arrStatus, VPI_LOCK_READ_WRITE, &statusdata);
    vpiArrayLock(v, VPI_LOCK_READ_WRITE, &arrData);
    arrPoints = (VPIKeypoint *)arrData.data;
    const uint8_t *status = (uint8_t *)statusdata.data;
    int totKeypoints = *arrData.sizePointer;
#else
    vpiArrayLockData(arrStatus, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &statusdata);
    vpiArrayLockData(v, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &arrData);
    arrPoints = (VPIKeypoint *)arrData.buffer.aos.data;
    const uint8_t *status = (uint8_t *)statusdata.buffer.aos.data;
    int totKeypoints = *arrData.buffer.aos.sizePointer;
#endif

    for (int i = 0; i < totKeypoints; i++)
    {
        if (1 - status[i])
        {
            arrPoints[j].x = arrPoints[i].x;
            arrPoints[j].y = arrPoints[i].y;
            j += 1;
        }
    }
#if NV_VPI_VERSION_MAJOR == 1
    *arrData.sizePointer = j;
#else
    *arrData.buffer.aos.sizePointer = j;
#endif
    vpiArrayUnlock(v);
}
void VPIFeatureTracker::reduceVectorVPI(VPIArray v, vector<uchar> &arrStatus)
{
    int j = 0;
    VPIArrayData arrData;
    VPIKeypoint *arrPoints;
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(v, VPI_LOCK_READ_WRITE, &arrData);
    arrPoints = (VPIKeypoint *)arrData.data;
#else
    vpiArrayLockData(v, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &arrData);
    arrPoints = (VPIKeypoint *)arrData.buffer.aos.data;
#endif
    for (int i = 0; i < int(arrStatus.size()); i++)
    {
        if (arrStatus[i])
        {
            arrPoints[j].x = arrPoints[i].x;
            arrPoints[j].y = arrPoints[i].y;
            j += 1;
        }
    }
#if NV_VPI_VERSION_MAJOR == 1
    *arrData.sizePointer = j;
#else
    *arrData.buffer.aos.sizePointer = j;
#endif
    vpiArrayUnlock(v);
}

void VPIFeatureTracker::reduceVector1(vector<cv::Point2f> &v, vector<uchar> status, VPIArray correspondingVPIarr)
{
    int j = 0;
    VPIArrayData arrData;
    VPIKeypoint *arrPoints;

    bool flag = false;
    if (correspondingVPIarr != NULL)
    {
#if NV_VPI_VERSION_MAJOR == 1
        vpiArrayLock(correspondingVPIarr, VPI_LOCK_READ_WRITE, &arrData);
        arrPoints = (VPIKeypoint *)arrData.data;
#else
        vpiArrayLockData(correspondingVPIarr, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &arrData);
        arrPoints = (VPIKeypoint *)arrData.buffer.aos.data;
#endif
        flag = true;
    }
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
        {
            v[j] = v[i];
            if (flag)
            {
                arrPoints[j].x = arrPoints[i].x;
                arrPoints[j].y = arrPoints[i].y;
            }
            j = j + 1;
        }
    v.resize(j);
    if (flag)
    {
#if NV_VPI_VERSION_MAJOR == 1
        *arrData.sizePointer = j;
#else
        *arrData.buffer.aos.sizePointer = j;
#endif
        vpiArrayUnlock(correspondingVPIarr);
    }
    // std::cerr<<"arr Size:"<<j<<std::endl;
}
void outputVPIKeypoints(VPIArray src)
{
    VPIArrayData srcdata;
#if NV_VPI_VERSION_MAJOR == 1
    CHECK_STATUS(vpiArrayLock(src, VPI_LOCK_READ, &srcdata));
    const VPIKeypoint *srcPoints = (VPIKeypoint *)srcdata.data;
    int totKeypoints = *srcdata.sizePointer;
#else
    vpiArrayLockData(src, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &srcdata);
    const VPIKeypoint *srcPoints = (VPIKeypoint *)srcdata.buffer.aos.data;
    int totKeypoints = *srcdata.buffer.aos.sizePointer;
#endif

    std::cerr << "output keypoint in VPI array" << std::endl;
    for (int i = 0; i < totKeypoints; i++)
    {
        std::cerr << "(" << srcPoints[i].x << "," << srcPoints[i].y << ")";
    }
    std::cerr << std::endl;
    CHECK_STATUS(vpiArrayUnlock(src));
}

void VPIFeatureTracker::reduceVectorVPI(vector<int> &v, VPIArray status)
{
    int j = 0;
    VPIArrayData statusdata;
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(status, VPI_LOCK_READ_WRITE, &statusdata);
    const uint8_t *status_data = (uint8_t *)statusdata.data;
    int tot = *statusdata.sizePointer;
#else
    vpiArrayLockData(status, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &statusdata);
    const uint8_t *status_data = (uint8_t *)statusdata.buffer.aos.data;
    int tot = *statusdata.buffer.aos.sizePointer;
#endif

    for (int i = 0; i < tot; i++)
        if (1 - status_data[i])
            v[j++] = v[i];
    v.resize(j);
    vpiArrayUnlock(status);
}

void VPIFeatureTracker::reduceVectorVPI2(vector<cv::Point2f> &v, VPIArray status)
{
    int j = 0;
    VPIArrayData statusdata;
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(status, VPI_LOCK_READ_WRITE, &statusdata);
    const uint8_t *status_data = (uint8_t *)statusdata.data;
    int tot = *statusdata.sizePointer;
#else
    vpiArrayLockData(status, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &statusdata);
    const uint8_t *status_data = (uint8_t *)statusdata.buffer.aos.data;
    int tot = *statusdata.buffer.aos.sizePointer;
#endif
    for (int i = 0; i < tot; i++)
        if (1 - status_data[i])
            v[j++] = v[i];
    v.resize(j);
    vpiArrayUnlock(status);
}

// void VPIFeatureTracker::reduceVector(vector<int> &v, vector<uchar> status)
// {
//     int j = 0;
//     for (int i = 0; i < int(v.size()); i++)
//         if (status[i])
//             v[j++] = v[i];
//     v.resize(j);
// }

void VPIKeyPointArr_to_cvPointVec(VPIArray &src, vector<cv::Point2f> &dst)
{
    // std::cerr<<"vpi keypoints arr to cv point vec"<<std::endl;
    VPIArrayData srcdata;
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(src, VPI_LOCK_READ, &srcdata);
    const VPIKeypoint *srcPoints = (VPIKeypoint *)srcdata.data;
    int totKeypoints = *srcdata.sizePointer;
#else
    vpiArrayLockData(src, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &srcdata);
    const VPIKeypoint *srcPoints = (VPIKeypoint *)srcdata.buffer.aos.data;
    int totKeypoints = *srcdata.buffer.aos.sizePointer;
#endif

    dst.resize(totKeypoints);
    for (int i = 0; i < totKeypoints; i++)
    {
        cv::Point2f srcPoint{srcPoints[i].x, srcPoints[i].y};
        dst[i] = srcPoint;
    }
    vpiArrayUnlock(src);
    // std::cerr<<"finish vpi keypoints arr to cv point vec"<<std::endl;
}
void VPIstatus_to_cvStatus(VPIArray &src, vector<uchar> &dst)
{
    VPIArrayData srcdata;
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(src, VPI_LOCK_READ, &srcdata);
    const uint8_t *srcStatus = (uint8_t *)srcdata.data;
    int size = *srcdata.sizePointer;
#else
    vpiArrayLockData(src, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &srcdata);
    const uint8_t *srcStatus = (uint8_t *)srcdata.buffer.aos.data;
    int size = *srcdata.buffer.aos.sizePointer;
#endif

    dst.resize(size);
    for (int i = 0; i < size; i++)
    {
        dst[i] = 1 - srcStatus[i];
        // if (srcStatus[i] == 0){
        //     dst[i] = 1;
        // }
        // else{
        //     dst[i] = 0;
        // }
        // dst[i] = srcStatus[i];
        // std::cerr<<int(dst[i]);
    }

    vpiArrayUnlock(src);
}

vector<cv::Point2f> VPIFeatureTracker::pickPtsByQuatTree(VPIArray src, VPIArray scores, std::size_t amount)
{
    // std::cerr<<"creating array data"<<std::endl;
    VPIArrayData srcdata;
    VPIArrayData scoresdata;
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(src, VPI_LOCK_READ, &srcdata);
    vpiArrayLock(scores, VPI_LOCK_READ, &scoresdata);
    const VPIKeypoint *srcPoints = (VPIKeypoint *)srcdata.data;
    const float *ptsScores = (float *)scoresdata.data;
    int totKeypoints = *srcdata.sizePointer;
#else
    vpiArrayLockData(src, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &srcdata);
    vpiArrayLockData(scores, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &scoresdata);
    const VPIKeypoint *srcPoints = (VPIKeypoint *)srcdata.buffer.aos.data;
    const float *ptsScores = (float *)scoresdata.buffer.aos.data;
    int totKeypoints = *srcdata.buffer.aos.sizePointer;
#endif
    vector<cv::Point2f> tmp(totKeypoints);
    vector<float> tmpscores(totKeypoints);
    for (int i = 0; i < totKeypoints; i++)
    {
        cv::Point2f srcPoint{srcPoints[i].x, srcPoints[i].y};
        tmp[i] = srcPoint;
        tmpscores[i] = ptsScores[i];
    }
    vpiArrayUnlock(src);
    vpiArrayUnlock(scores);
    //  std::cerr<<"finish init"<<std::endl;
    if (totKeypoints < amount)
    {
        return tmp;
    }
    // std::cerr<<"creating the first node"<<std::endl;
    node firstNode;
    firstNode.beginIdx = 0;
    firstNode.endIdx = totKeypoints; // next one of the last index
    firstNode.minCol = 0;
    firstNode.minRow = 0;
    firstNode.maxCol = cam->COL;
    firstNode.maxRow = cam->ROW;
    vector<node> nodes;
    nodes.push_back(firstNode);
    while (nodes.size() < amount)
    {
        // std::cerr<<"amount:"<<amount<<","<<"current node number:"<<nodes.size()<<std::endl;
        nodes = splitNode(tmp, tmpscores, nodes);
    }
    // pick the points in each node with the highest score
    vector<pair<cv::Point2f, float>> n_pts_score;
    for (auto &it : nodes)
    {
        int maxIdx = it.beginIdx;
        for (int i = it.beginIdx; i < it.endIdx; i++)
        {
            if (tmpscores[i] > tmpscores[maxIdx])
            {
                maxIdx = i;
            }
        }
        n_pts_score.push_back(std::make_pair(tmp[maxIdx], tmpscores[maxIdx]));
    }
#if quadTreePickStrategy
    std::partial_sort(n_pts_score.begin(), n_pts_score.begin() + amount, n_pts_score.end(), [](const pair<cv::Point2f, float> &a, const pair<cv::Point2f, float> &b)
                      { return a.second >= b.second; });
    vector<cv::Point2f> n_pts;
    for (int i = 0; i < amount; i++)
    {
        n_pts.push_back(n_pts_score[i].first);
    }
    return n_pts;
#else
    vector<cv::Point2f> n_pts;
    float stride = (float)nodes.size() / (float)amount;
    float idx = 0;
    for (int i = 0; i < amount; i++)
    {
        n_pts.push_back(n_pts_score[(int)idx % amount].first);
        idx += stride;
    }
    return n_pts;
#endif
}

vector<node> splitNode(vector<cv::Point2f> &v, vector<float> &scores, vector<node> &info)
{
    // for each node, split it into four child nodes
    vector<node> newNodes;
    while (info.size() > 0)
    {
        node pNode = info[info.size() - 1];
        info.pop_back();
        if (pNode.endIdx - pNode.beginIdx <= 1)
        {
            newNodes.emplace_back(pNode);
            continue;
        }
        vector<cv::Point2f> ul;
        vector<float> score_ul;
        vector<cv::Point2f> ur;
        vector<float> score_ur;
        vector<cv::Point2f> bl;
        vector<float> score_bl;
        vector<cv::Point2f> br;
        vector<float> score_br;
        int splitCol = (pNode.minCol + pNode.maxCol) / 2;
        int splitRow = (pNode.minRow + pNode.maxRow) / 2;
        for (int i = pNode.beginIdx; i < pNode.endIdx; i++)
        {
            if (v[i].y <= splitRow)
            {
                if (v[i].x <= splitCol)
                {
                    ul.emplace_back(v[i]);
                    score_ul.emplace_back(scores[i]);
                }
                else
                {
                    ur.emplace_back(v[i]);
                    score_ur.emplace_back(scores[i]);
                }
            }
            else
            {
                if (v[i].x <= splitCol)
                {
                    bl.emplace_back(v[i]);
                    score_bl.emplace_back(scores[i]);
                }
                else
                {
                    br.emplace_back(v[i]);
                    score_br.emplace_back(scores[i]);
                }
            }
        }
        node ulnode;
        ulnode.beginIdx = pNode.beginIdx;
        ulnode.endIdx = ulnode.beginIdx + ul.size();
        // std::cerr<<"ul node size:"<<ul.size()<<std::endl;
        // std::cerr<<"["<<ulnode.beginIdx<<","<<ulnode.endIdx<<")"<<std::endl;
        if (ul.size() != 0)
        {
            ulnode.minRow = pNode.minRow;
            ulnode.minCol = pNode.minCol;
            ulnode.maxRow = splitRow;
            ulnode.maxCol = splitCol;
            newNodes.emplace_back(ulnode);
            // rearrange node sequence
            for (int i = ulnode.beginIdx; i < ulnode.endIdx; i++)
            {
                v[i] = ul[i - ulnode.beginIdx];
                scores[i] = score_ul[i - ulnode.beginIdx];
            }
        }
        node urnode;
        urnode.beginIdx = ulnode.endIdx;
        urnode.endIdx = urnode.beginIdx + ur.size();
        // std::cerr<<"ur node size:"<<ur.size()<<std::endl;
        // std::cerr<<"["<<urnode.beginIdx<<","<<urnode.endIdx<<")"<<std::endl;
        if (ur.size() != 0)
        {
            urnode.minRow = pNode.minRow;
            urnode.minCol = splitCol;
            urnode.maxRow = splitRow;
            urnode.maxCol = pNode.maxCol;
            newNodes.emplace_back(urnode);
            for (int i = urnode.beginIdx; i < urnode.endIdx; i++)
            {
                v[i] = ur[i - urnode.beginIdx];
                scores[i] = score_ur[i - urnode.beginIdx];
            }
        }
        node blnode;
        blnode.beginIdx = urnode.endIdx;
        blnode.endIdx = blnode.beginIdx + bl.size();
        //         std::cerr<<"bl node size:"<<bl.size()<<std::endl;
        // std::cerr<<"["<<blnode.beginIdx<<","<<blnode.endIdx<<")"<<std::endl;
        if (bl.size() != 0)
        {
            blnode.minRow = splitRow;
            blnode.minCol = pNode.minCol;
            blnode.maxRow = pNode.maxRow;
            blnode.maxCol = splitCol;
            newNodes.emplace_back(blnode);
            for (int i = blnode.beginIdx; i < blnode.endIdx; i++)
            {
                v[i] = bl[i - blnode.beginIdx];
                scores[i] = score_bl[i - blnode.beginIdx];
            }
        }
        node brnode;
        brnode.beginIdx = blnode.endIdx;
        brnode.endIdx = pNode.endIdx; // brnode.beginIdx + br.size()
        // std::cerr<<"br node size:"<<br.size()<<std::endl;
        // std::cerr<<"["<<brnode.beginIdx<<","<<brnode.endIdx<<")"<<std::endl;
        if (br.size() != 0)
        {
            brnode.minCol = splitCol;
            brnode.minRow = splitRow;
            brnode.maxCol = pNode.maxCol;
            brnode.maxRow = pNode.maxRow;
            newNodes.emplace_back(brnode);
            for (int i = brnode.beginIdx; i < brnode.endIdx; i++)
            {
                v[i] = br[i - brnode.beginIdx];
                scores[i] = score_br[i - brnode.beginIdx];
            }
        }
    }
    return newNodes;
}

VPIFeatureTracker::VPIFeatureTracker() : TrackerBase()
{
    backend = VPI_BACKEND_CUDA;
    // pyrLevel = 5;
    pyrLevel = 3;
    round = 0;
}
void SortKeypoints(VPIArray keypoints, VPIArray scores, std::size_t max)
{
    VPIArrayData ptsData, scoresData;
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(keypoints, VPI_LOCK_READ_WRITE, &ptsData);
    vpiArrayLock(scores, VPI_LOCK_READ_WRITE, &scoresData);
    std::vector<int> indices(*ptsData.sizePointer);
#else
    vpiArrayLockData(keypoints, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &ptsData);
    vpiArrayLockData(scores, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &scoresData);
    std::vector<int> indices(*ptsData.buffer.aos.sizePointer);
#endif
    std::iota(indices.begin(), indices.end(), 0);
    max = max < indices.size() ? max : indices.size();
    // only need to sort the first 'max' keypoints
    // report error when the actural size smaller than bound
    std::partial_sort(indices.begin(), indices.begin() + max, indices.end(), [&scoresData](int a, int b)
                      {
#if NV_VPI_VERSION_MAJOR == 1
        uint32_t *score = reinterpret_cast<uint32_t *>(scoresData.data);
#else
        uint32_t *score = reinterpret_cast<uint32_t *>(scoresData.buffer.aos.data);
#endif
        return score[a] >= score[b]; });
    // keep the only 'max' indexes.
    indices.resize(std::min<size_t>(indices.size(), max));
#if NV_VPI_VERSION_MAJOR == 1
    VPIKeypoint *kptData = reinterpret_cast<VPIKeypoint *>(ptsData.data);
#else
    VPIKeypoint *kptData = reinterpret_cast<VPIKeypoint *>(ptsData.buffer.aos.data);
#endif
    std::vector<VPIKeypoint> kpt;
    std::transform(indices.begin(), indices.end(), std::back_inserter(kpt),
                   [kptData](int idx)
                   { return kptData[idx]; });
    std::copy(kpt.begin(), kpt.end(), kptData);

    // update keypoint array size.
#if NV_VPI_VERSION_MAJOR == 1
    *ptsData.sizePointer = kpt.size();
#else
    *ptsData.buffer.aos.sizePointer = kpt.size();
#endif
    vpiArrayUnlock(scores);
    vpiArrayUnlock(keypoints);
}

void VPIFeatureTracker::setMask()
{
    if (cam->FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(cam->ROW, cam->COL, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    // vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    vector<pair<int, pair<VPIKeypoint, int>>> cnt_pts_id;
    VPIArrayData forwArrData;
    VPIKeypoint *forwArrPoints;
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(arrForwPts, VPI_LOCK_READ_WRITE, &forwArrData);
    forwArrPoints = (VPIKeypoint *)forwArrData.data;
    int totKeypoints = *forwArrData.sizePointer;
#else
    vpiArrayLockData(arrForwPts, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &forwArrData);
    forwArrPoints = (VPIKeypoint *)forwArrData.buffer.aos.data;
    int totKeypoints = *forwArrData.buffer.aos.sizePointer;
#endif
    // std::cerr<<totKeypoints<<std::endl;
    for (unsigned int i = 0; i < totKeypoints; i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forwArrPoints[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<VPIKeypoint, int>> &a, const pair<int, pair<VPIKeypoint, int>> &b)
         { return a.first > b.first; });
    vpiArrayUnlock(arrForwPts);

    // forw_pts.clear();
    vpiArrayDestroy(arrForwPts);
    vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &arrForwPts);
    ids.clear();
    track_cnt.clear();
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(arrForwPts, VPI_LOCK_READ_WRITE, &forwArrData);
    forwArrPoints = (VPIKeypoint *)forwArrData.data;
#else
    vpiArrayLockData(arrForwPts, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &forwArrData);
    forwArrPoints = (VPIKeypoint *)forwArrData.buffer.aos.data;
#endif
    int i = 0;
    // std::cerr<<cnt_pts_id.size()<<std::endl;
    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(cv::Point2f(it.second.first.x, it.second.first.y)) == 255)
        {
            // forw_pts.push_back(it.second.first);
            forwArrPoints[i].x = it.second.first.x;
            forwArrPoints[i].y = it.second.first.y;
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::Point2f draw_point{it.second.first.x, it.second.first.y};
            cv::circle(mask, draw_point, cam->MIN_DIST, 0, -1);
            i = i + 1;
        }
    }
#if NV_VPI_VERSION_MAJOR == 1
    *forwArrData.sizePointer = i;
#else
    *forwArrData.buffer.aos.sizePointer = i;
#endif
    vpiArrayUnlock(arrForwPts);
    // vpiArraySetSize(arrForwPts,i);
    // std::cerr<<"output track_cnt:"<<track_cnt.size()<<std::endl;
    //     for (auto &i:track_cnt) std::cerr<<i<<",";
    // std::cerr<<std::endl;
}

void VPIFeatureTracker::addPoints()
{
    VPIArrayData forwArrData;
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(arrForwPts, VPI_LOCK_READ_WRITE, &forwArrData);
    VPIKeypoint *forwArrPoints = (VPIKeypoint *)forwArrData.data;
#else
    vpiArrayLockData(arrForwPts, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &forwArrData);
    VPIKeypoint *forwArrPoints = (VPIKeypoint *)forwArrData.buffer.aos.data;
#endif
    // std::cerr<<"n_pts size:"<<n_pts.size()<<std::endl;
    for (auto &p : n_pts)
    {
        // forw_pts.push_back(p);
        // add arrforwpts
#if NV_VPI_VERSION_MAJOR == 1
        int32_t end_idx = *forwArrData.sizePointer;
        *forwArrData.sizePointer = *forwArrData.sizePointer + 1;
#else
        int32_t end_idx = *forwArrData.buffer.aos.sizePointer;
        *forwArrData.buffer.aos.sizePointer = *forwArrData.buffer.aos.sizePointer + 1;
#endif
        VPIKeypoint new_p;
        new_p.x = p.x;
        new_p.y = p.y;
        forwArrPoints[end_idx] = new_p;
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
    // std::cerr<<*forwArrData.sizePointer<<std::endl;
    vpiArrayUnlock(arrForwPts);
}

void VPIFeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    // std::cerr<<"round:"<<round<<std::endl;
    // round = round + 1;
    // if (round<450) return;
    LOG(INFO) << "VPI read image";
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (cam->EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        // ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    // forw_pts.clear();
    vpiArrayDestroy(arrForwPts);
    vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &arrForwPts);
    vpiImageSetWrappedOpenCVMat(VPI_forw_img, img);
    int cur_pts_num;
    vpiArrayGetSize(arrCurPts, &cur_pts_num);
    if (cur_pts_num > 0)
    {
        TicToc t_o;
        // vector<uchar> status;

        vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_U8, 0, &arrStatus);
// CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &arrForwPts));
// vector<float> err;
//  std::cerr<<"cur_pts size"<<cur_pts.size()<<std::endl;
//  Optical FLow Pyr LK replaced by VPI
// cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
// std::cerr<<"wrap img"<<std::endl;
#if NV_VPI_VERSION_MAJOR == 1
        vpiSubmitGaussianPyramidGenerator(stream, backend, VPI_forw_img, pyrForwFrame);
#else
        vpiSubmitGaussianPyramidGenerator(stream, backend, VPI_forw_img, pyrForwFrame, VPI_BORDER_ZERO);
#endif
        // std::cerr<<"submit LK"<<std::endl;
        // outputVPIKeypoints(arrCurPts);
        // std::cerr<<"submit LK"<<std::endl;
        CHECK_STATUS(vpiSubmitOpticalFlowPyrLK(stream, backend, optflow, pyrCurFrame, pyrForwFrame, arrCurPts, arrForwPts, arrStatus, &lkParams));
        // VPI status to vector status
        // std::cerr<<"forw pts"<<std::endl;
        // outputVPIKeypoints(arrForwPts);
        vpiStreamSync(stream);

        VPIArrayData forw_pts_data, status_data;
#if NV_VPI_VERSION_MAJOR == 1
        vpiArrayLock(arrForwPts, VPI_LOCK_READ_WRITE, &forw_pts_data);
        vpiArrayLock(arrStatus, VPI_LOCK_READ_WRITE, &status_data);
        VPIKeypoint *forw_points = (VPIKeypoint *)forw_pts_data.data;
        uint8_t *status_data_ = (uint8_t *)status_data.data;
        int totKeypoints = *forw_pts_data.sizePointer;
        int status_size = *status_data.sizePointer;
#else
        vpiArrayLockData(arrForwPts, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &forw_pts_data);
        vpiArrayLockData(arrStatus, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &status_data);
        VPIKeypoint *forw_points = (VPIKeypoint *)forw_pts_data.buffer.aos.data;
        uint8_t *status_data_ = (uint8_t *)status_data.buffer.aos.data;
        int totKeypoints = *forw_pts_data.buffer.aos.sizePointer;
        int status_size = *status_data.buffer.aos.sizePointer;
#endif
        // std::cerr<<"tracked count:"<<totKeypoints<<","<<status_size<<std::endl;
        for (int i = 0; i < totKeypoints; i++)
        {
            if ((1 - status_data_[i]) && !inBorder(forw_points[i]))
                status_data_[i] = 1;
        }

        vpiArrayUnlock(arrForwPts);
        vpiArrayUnlock(arrStatus);

        // reduceVector1(prev_pts, status,arrPrevPts);
        reduceVectorVPI(arrPrevPts, arrStatus);
        // reduceVector1(cur_pts, status,arrCurPts);
        reduceVectorVPI(arrCurPts, arrStatus);
        // reduceVector1(forw_pts, status,arrForwPts);
        reduceVectorVPI(arrForwPts, arrStatus);
        // reduceVector(ids, status);
        reduceVectorVPI(ids, arrStatus);
        // reduceVector1(cur_un_pts, status);
        reduceVectorVPI2(cur_un_pts, arrStatus);
        // reduceVector(track_cnt, status);
        reduceVectorVPI(track_cnt, arrStatus);
        vpiArrayDestroy(arrStatus);
        // std::cerr<<std::endl<<"finish reduce"<<std::endl;
        // ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;

    if (cam->PUB_THIS_FRAME)
    {
        // std::cout<<"pub this frame:"<<round++<<std::endl;
        rejectWithF();
        // ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        // ROS_DEBUG("set mask costs %fms", t_m.toc());

        // ROS_DEBUG("detect feature begins");
        TicToc t_t;
        // VPIArrayData tmp_data;
        // vpiArrayLock(arrForwPts,VPI_LOCK_READ,&tmp_data);
        int totKeypoints;
        vpiArrayGetSize(arrForwPts, &totKeypoints);
        int n_max_cnt = cam->MAX_CNT - static_cast<int>(totKeypoints);
        // vpiArrayUnlock(arrForwPts);
        // std::cerr<<"n_max_cnt:"<<n_max_cnt<<std::endl;
        if (n_max_cnt > 0)
        {
            if (mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            // replaced by VPI
            //  std::cerr<<"create array for harris"<<std::endl;
            //  std::cerr<<"size:"<<MAX_CNT - forw_pts.size()<<std::endl;
            vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &VPI_n_pts);
            vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_U32, 0, &scores);
            // std::cerr<<"finish create"<<std::endl;
            //  std::cerr<<"submit harris corner detector"<<std::endl;
            vpiSubmitHarrisCornerDetector(stream, backend, harris, VPI_forw_img, VPI_n_pts, scores, &harrisParams);
            vpiStreamSync(stream);
            // std::cerr<<"sort"<<std::endl;
            // SortKeypoints(VPI_n_pts, scores, MAX_KEYPOINTS);
            // std::cerr<<"finish sort"<<std::endl;
            if (cam->MAX_CNT - totKeypoints == 1)
            {
                // std::cerr<<"add only one point"<<std::endl;
                SortKeypoints(VPI_n_pts, scores, 1);
                VPIKeyPointArr_to_cvPointVec(VPI_n_pts, n_pts);
            }
            else
            {
                // std::cerr<<"start using quat tree"<<std::endl;
                n_pts = pickPtsByQuatTree(VPI_n_pts, scores, cam->MAX_CNT - totKeypoints);
                // std::cerr<<"finish using quad tree"<<std::endl;
            }

            // cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
            vpiArrayDestroy(VPI_n_pts);
            vpiArrayDestroy(scores);
        }
        else
        {
            // std::cerr<<"clear n_pts"<<std::endl;
            n_pts.clear();

            // CHECK_STATUS(vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &VPI_n_pts));
        }
        // ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        // ROS_DEBUG("add feature begins");
        TicToc t_a;
        // std::cerr<<"add points"<<std::endl;
        addPoints();
        // ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    // std::cerr<<"swapping"<<std::endl;
    prev_img = cur_img;
    // prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    // cur_pts = forw_pts;
    std::swap(VPI_prev_img, VPI_cur_img);
    std::swap(VPI_cur_img, VPI_forw_img);
    std::swap(arrPrevPts, arrCurPts);
    std::swap(arrCurPts, arrForwPts);
    std::swap(pyrCurFrame, pyrForwFrame);
    undistortedPoints();
    // std::cerr<<"finish undistorted"<<std::endl;
    prev_time = cur_time;
}

void VPIFeatureTracker::rejectWithF()
{
    int forw_pts_size;
    vpiArrayGetSize(arrForwPts, &forw_pts_size);

    if (forw_pts_size >= 8)
    {
        //  std::cerr<<"reject with F"<<std::endl;
        // ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        VPIArrayData cur_data, forw_data;
#if NV_VPI_VERSION_MAJOR == 1
        vpiArrayLock(arrCurPts, VPI_LOCK_READ, &cur_data);
        vpiArrayLock(arrForwPts, VPI_LOCK_READ, &forw_data);
        VPIKeypoint *cur_data_pts;
        VPIKeypoint *forw_data_pts;
        cur_data_pts = (VPIKeypoint *)cur_data.data;
        forw_data_pts = (VPIKeypoint *)forw_data.data;
        int cur_pts_num = *cur_data.sizePointer;
        int forw_pts_num = *forw_data.sizePointer;
#else
        vpiArrayLockData(arrCurPts, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &cur_data);
        vpiArrayLockData(arrForwPts, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &forw_data);
        VPIKeypoint *cur_data_pts;
        VPIKeypoint *forw_data_pts;
        cur_data_pts = (VPIKeypoint *)cur_data.buffer.aos.data;
        forw_data_pts = (VPIKeypoint *)forw_data.buffer.aos.data;
        int cur_pts_num = *cur_data.buffer.aos.sizePointer;
        int forw_pts_num = *forw_data.buffer.aos.sizePointer;
#endif
        vector<uchar> status;
        // #if USE_GPU
        //     float un_cur_pts[cur_pts_num*2];
        //     float un_forw_pts[forw_pts_num*2];
        //     for (unsigned int i = 0; i < cur_pts_num; i++)
        //     {
        //         Eigen::Vector3d tmp_p;
        //         m_camera->liftProjective(Eigen::Vector2d(cur_data_pts[i].x, cur_data_pts[i].y), tmp_p);
        //         tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
        //         tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
        //         un_cur_pts[i] = tmp_p.x();
        //         un_cur_pts[cur_pts_num+i] = tmp_p.y();

        //         m_camera->liftProjective(Eigen::Vector2d(forw_data_pts[i].x, forw_data_pts[i].y), tmp_p);
        //         tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
        //         tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
        //         un_forw_pts[i] = tmp_p.x();
        //         un_forw_pts[forw_pts_num+i] = tmp_p.y();
        //     }
        //     vpiArrayUnlock(arrCurPts);
        //     vpiArrayUnlock(arrForwPts);
        //     RANSAC_cuda_tools::findFundamentalMat_on_cuda_array_input(un_cur_pts,un_forw_pts,cur_pts_num,0.02,0.99,status);
        //     // RANSAC_cuda_tools::findFundamentalMat_on_cuda(un_cur_pts,un_forw_pts,0.05,0.99,status)
        // #else
        vector<cv::Point2f> un_cur_pts(cur_pts_num), un_forw_pts(forw_pts_num);
        for (unsigned int i = 0; i < cur_pts_num; i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_data_pts[i].x, cur_data_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + cam->COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + cam->ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_data_pts[i].x, forw_data_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + cam->COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + cam->ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }
        vpiArrayUnlock(arrCurPts);
        vpiArrayUnlock(arrForwPts);
#if USE_GPU
        RANSAC_cuda_tools::findFundamentalMat_on_cuda(un_cur_pts, un_forw_pts, 0.05, 0.99, status);
#else
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, cam->F_THRESHOLD, 0.99, status);
#endif

        // string filenamebase = "/home/yao/pts_test_vpi";
        // string filename = filenamebase + "/pts" + to_string(filecount)+".csv";
        // filecount++;
        // ofstream ofile;
        // std::cout<<"start ransac on cuda"<<std::endl;
        // std::cerr<<"forw_pts size"<<un_forw_pts.size()<<","<<"cur pts size"<<un_cur_pts.size()<<std::endl;

        // ofile.open(filename);
        // format: cur pts, forw pts, status
        // for (int i=0;i<un_cur_pts.size();i++){
        //     ofile<<un_cur_pts[i].x<<','<<un_cur_pts[i].y<<','<<un_forw_pts[i].x<<','<<un_forw_pts[i].y<<','<<(int)status[i]<<'\n';
        // }
        // ofile.close();
        // int size_a = cur_pts.size();
        // std::cerr<<"in reject---------------"<<std::endl;
        // std::cerr<<"status:"<<status.size()<<std::endl;
        // std::cerr<<"cur pts:"<<cur_pts.size()<<std::endl;
        // std::cerr<<"forw pts:"<<forw_pts.size()<<std::endl;
        // std::cerr<<"prev pts:"<<prev_pts.size()<<std::endl;
        // std::cerr<<"start reduce"<<std::endl;
        // reduceVector1(prev_pts, status,arrPrevPts);//reduce vector apply also on VPI array
        reduceVectorVPI(arrPrevPts, status);
        // reduceVector1(cur_pts, status,arrCurPts);
        reduceVectorVPI(arrCurPts, status);
        // reduceVector1(forw_pts, status,arrForwPts);
        reduceVectorVPI(arrForwPts, status);
        reduceVector1(cur_un_pts, status);
        MCVIO::TrackerBase::reduceVector(ids, status);
        MCVIO::TrackerBase::reduceVector(track_cnt, status);
        // std::cerr<<"start reduce"<<std::endl;
        // ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        // ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

bool VPIFeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void VPIFeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    // ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void VPIFeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(cam->ROW + 600, cam->COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < cam->COL; i++)
        for (int j = 0; j < cam->ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            // printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + cam->COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + cam->ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        // cout << trackerData[0].K << endl;
        // printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        // printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < cam->ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < cam->COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            // ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void VPIFeatureTracker::undistortedPoints()
{
    // std::cerr<<"undistorted"<<std::endl;
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    VPIArrayData cur_data;
    VPIKeypoint *cur_data_pts;
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(arrCurPts, VPI_LOCK_READ_WRITE, &cur_data);
    cur_data_pts = (VPIKeypoint *)cur_data.data;
    int totKeypoints = *cur_data.sizePointer;
#else
    vpiArrayLockData(arrCurPts, VPI_LOCK_READ_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &cur_data);
    cur_data_pts = (VPIKeypoint *)cur_data.buffer.aos.data;
    int totKeypoints = *cur_data.buffer.aos.sizePointer;
#endif
    // cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < totKeypoints; i++)
    {
        Eigen::Vector2d a(cur_data_pts[i].x, cur_data_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        // printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    vpiArrayUnlock(arrCurPts);
    ROS_WARN("Current un pts size: %d, Prev un pts size: %d", (int)cur_un_pts.size(), (int)prev_un_pts_map.size());
    int count = 0;
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    count++;
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < totKeypoints; i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    ROS_WARN("matching rate: %f", 1.0 * count / (int)cur_un_pts.size());
    prev_un_pts_map = cur_un_pts_map;
}

// initialize VPI data structure
void VPIFeatureTracker::initVPIData(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat init_img = ptr->image.rowRange(0, cam->ROW);
    vpiStreamCreate(0, &stream);
// CV mat wrapper
#if NV_VPI_VERSION_MAJOR == 1
    vpiImageCreateOpenCVMatWrapper(init_img, 0, &VPI_prev_img);
    vpiImageCreateOpenCVMatWrapper(init_img, 0, &VPI_cur_img);
    vpiImageCreateOpenCVMatWrapper(init_img, 0, &VPI_forw_img);
#else
    vpiImageCreateWrapperOpenCVMat(init_img, 0, &VPI_prev_img);
    vpiImageCreateWrapperOpenCVMat(init_img, 0, &VPI_cur_img);
    vpiImageCreateWrapperOpenCVMat(init_img, 0, &VPI_forw_img);
#endif
    // get format
    vpiImageGetFormat(VPI_prev_img, &imgFormat);
    // create pyramid
    float downScale = 0.5; // default = 0.5
    vpiPyramidCreate(init_img.cols, init_img.rows, imgFormat, pyrLevel, downScale, 0, &pyrForwFrame);
    vpiPyramidCreate(init_img.cols, init_img.rows, imgFormat, pyrLevel, downScale, 0, &pyrCurFrame);
    // create array
    vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &arrPrevPts);
    vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &arrCurPts);
    vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_KEYPOINT, 0, &arrForwPts);
    vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_U8, 0, &arrStatus);
    vpiArrayCreate(MAX_HARRIS_CORNERS, VPI_ARRAY_TYPE_U32, 0, &scores);
    // create optical flow pyramidal LK
    vpiCreateOpticalFlowPyrLK(backend, init_img.cols, init_img.rows, imgFormat, pyrLevel, downScale, &optflow);
    // set LK flow parameter
    memset(&lkParams, 0, sizeof(lkParams));
    lkParams.useInitialFlow = false;
    lkParams.termination = VPI_TERMINATION_CRITERIA_ITERATIONS | VPI_TERMINATION_CRITERIA_EPSILON;
    lkParams.epsilonType = VPI_LK_ERROR_L1;
    // lkParams.epsilon         = 0.0f;
    lkParams.epsilon = 0.01f;
    // lkParams.windowDimension = 15;
    lkParams.windowDimension = 21;
    // lkParams.numIterations   = 6;
    lkParams.numIterations = 12;
    // create the payload for harris corners detector
    vpiCreateHarrisCornerDetector(backend, init_img.cols, init_img.rows, &harris);

    harrisParams.gradientSize = 3;
    harrisParams.blockSize = 3;
    harrisParams.sensitivity = 0.06;
    harrisParams.minNMSDistance = 20;
    harrisParams.strengthThresh = 0.01;

// harrisParams.gradientSize   = 5;
// harrisParams.blockSize      = 5;
// harrisParams.blockSize = 7;
// harrisParams.sensitivity    = 0.01;
// harrisParams.minNMSDistance = 12;
// CHECK_STATUS(vpiSubmitHarrisCornerDetector(stream, backend, harris, VPI_cur_img, arrCurPts, scores, &harrisParams));
// CHECK_STATUS(vpiStreamSync(stream));
#if NV_VPI_VERSION_MAJOR == 1
    vpiSubmitGaussianPyramidGenerator(stream, backend, VPI_cur_img, pyrCurFrame);
#else
    vpiSubmitGaussianPyramidGenerator(stream, backend, VPI_cur_img, pyrCurFrame, VPI_BORDER_ZERO);
#endif
}

void VPIFeatureTracker::getPt(int idx, int &id, geometry_msgs::Point32 &p, cv::Point2f &p_uv, cv::Point2f &v)
{
    id = ids[idx];
    p.x = cur_un_pts[idx].x;
    p.y = cur_un_pts[idx].y;
    p.z = 1;
    v = pts_velocity[idx];
    p_uv.x = cur_data_pts[idx].x;
    p_uv.y = cur_data_pts[idx].y;
}

void VPIFeatureTracker::getCurPt(int idx, cv::Point2f &cur_pt)
{
    cur_pt.x = cur_data_pts[idx].x;
    cur_pt.y = cur_data_pts[idx].y;
}

void VPIFeatureTracker::Lock()
{
#if NV_VPI_VERSION_MAJOR == 1
    vpiArrayLock(arrCurPts, VPI_LOCK_READ, &cur_data);
    cur_data_pts = (VPIKeypoint *)cur_data.data;
#else
    vpiArrayLockData(arrCurPts, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &cur_data);
    cur_data_pts = (VPIKeypoint *)cur_data.buffer.aos.data;
#endif
}

void VPIFeatureTracker::Unlock()
{
    vpiArrayUnlock(arrCurPts);
}