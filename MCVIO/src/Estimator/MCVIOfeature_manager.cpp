#include "MCVIOfeature_manager.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

FeatureManager::FeatureManager()
{
}

FeatureManager::FeatureManager(Matrix3d _Rs[], MCVIOfrontend *frontend)
    : Rs(_Rs), frontend_(frontend)
{

    last_track_num.resize(NUM_OF_CAM, 0);
    KeyPointLandmarks.resize(NUM_OF_CAM);
    // for (int i = 0; i < NUM_OF_CAM; i++)
    //     ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    ric = (Matrix3d *)malloc(sizeof(Matrix3d) * NUM_OF_CAM);
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    LOG(INFO) << "Feature Manager clear state";
    if (KeyPointLandmarks.size() == 0)
        return;
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        KeyPointLandmarks[c].clear();
    }
}

int FeatureManager::getFeatureCount(int c)
{
    // LOG(INFO)<<"Get feature count for cam:" << c;
    int cnt = 0;
    // LOG(INFO)<<"KeyPointLandmarks size: "<<KeyPointLandmarks.size();

    for (auto &landmark : KeyPointLandmarks[c])
    {
        // LOG(INFO)<<"id: "<<landmark.first;
        landmark.second.used_num = landmark.second.obs.size();

        if (landmark.second.used_num >= 2 and time_frameid2_int_frameid_->at(landmark.second.kf_id) < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    // LOG(INFO)<<cnt;

    return cnt;
}

bool FeatureManager::addFeatureCheckParallax(int frame_count,
                                             const MCVIO::TimeFrameId frame_id,
                                             const MCVIO::SyncCameraProcessingResults &image,
                                             double td)
{
    // ROS_DEBUG("input feature: %d", (int)image.size());
    // ROS_DEBUG("num of feature: %d", getFeatureCount());
    printf("----\n File: \"%s\", line: %d, function <%s>\n ----", __FILE__, __LINE__, __func__);
    LOG(INFO) << "NUM_OF_CAM: " << NUM_OF_CAM;
    double parallax_sum[NUM_OF_CAM];
    double parallax_num[NUM_OF_CAM];
    // last_track_num = 0;

    int old_feature[NUM_OF_CAM];
    int new_feature[NUM_OF_CAM];
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        LOG(INFO) << "Process camera: " << c;
        LOG(INFO) << "Feature num:" << image.results[c]->features.size();
        parallax_sum[c] = 0;
        parallax_num[c] = 0;
        old_feature[c] = 0;
        new_feature[c] = 0;
        last_track_num[c] = 0;
        for (auto &id_pts : image.results[c]->features)
        {
            // id_pts.second[0].second: Eigen::Matrix<double, 8, 1>
            KeypointObservation kpt_obs(id_pts.second[0], td);

            MCVIO::FeatureID feature_id = id_pts.first;
            // printf("----\n File: \"%s\" \n line: %d \n function <%s>\n Content: %d : %d \n  =====",
            //        __FILE__, __LINE__, __func__,
            //        c, feature_id);
            if (KeyPointLandmarks[c].find(feature_id) == KeyPointLandmarks[c].end())
            {

                // stored measured depth in the first frame
                KeyPointLandmarks[c][feature_id] =
                    KeyPointLandmark(feature_id, frame_id, id_pts.second[0](7));

                CHECK(frame_id != 0) << "frame_id == 0";
                KeyPointLandmarks[c][feature_id].obs[frame_id] = kpt_obs;
                KeyPointLandmarks[c][feature_id].kf_id = frame_id;
                new_feature[c]++;
            }
            else
            {
                CHECK(frame_id != 0) << "frame_id == 0";
                KeyPointLandmarks[c][feature_id].obs[frame_id] = kpt_obs;
                last_track_num[c]++;
                old_feature[c]++;
            }
        }
    }
    printf("----\n File: \"%s\" \n line: %d \n function <%s>\n Content: %s \n  =====",
           __FILE__, __LINE__, __func__,
           "Check parallax");

    bool flag = false;
    //  LOG(INFO) << "old feature: " << old_feature << " new feature: " << new_feature;
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
#if SINGLE_CAM_DEBUG
        if (c > 0)
            continue;
#endif
        if (image.isEmpty[c])
        {
            continue;
        }
        if (frame_count < 2 || last_track_num[c] < 20)
            return true;

        CHECK(frame_count >= 2) << "frame size < 2";
        CHECK(int_frameid2_time_frameid_->size() >= 2) << "size: " << int_frameid2_time_frameid_->size();
        auto target1_tid = int_frameid2_time_frameid_->at(frame_count - 2);
        auto target2_tid = int_frameid2_time_frameid_->at(frame_count - 1);

        for (const auto &landmark : KeyPointLandmarks[c])
        {

            if (time_frameid2_int_frameid_->at(landmark.second.kf_id) <= frame_count - 2 and
                (time_frameid2_int_frameid_->at(landmark.second.kf_id) + int(landmark.second.obs.size()) - 1) >= frame_count - 1)
            {
                // ROS_DEBUG("it_per_id.start_frame: %d, tracked_num: %d",time_frameid2_int_frameid_->at(landmark.second.kf_id)
                // ,int(landmark.second.obs.size())-1);

                parallax_sum[c] += compensatedParallax2(landmark.second, frame_count);
                parallax_num[c]++;
            }

            // if (landmark.second.obs.find(target1_tid) != landmark.second.obs.end() &&
            //     landmark.second.obs.find(target2_tid) != landmark.second.obs.end()) {
            //     parallax_sum += compensatedParallax2(landmark.second, frame_count);
            //     parallax_num++;
            // }
        }

        if (parallax_num[c] == 0)
        {
            return true;
        }
        else
        {
            ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum[c], parallax_num[c]);
            ROS_DEBUG("current parallax: %lf", parallax_sum[c] / parallax_num[c] * FOCAL_LENGTH);
            flag = parallax_sum[c] / parallax_num[c] >= MIN_PARALLAX;
        }
    }
    return flag;
}

Eigen::aligned_vector<Eigen::aligned_vector<pair<Vector3d, Vector3d>>>
FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    Eigen::aligned_vector<Eigen::aligned_vector<pair<Vector3d, Vector3d>>> corres;
    corres.resize(NUM_OF_CAM);
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        for (const auto &landmark : KeyPointLandmarks[c])
        {
            auto i_tid = int_frameid2_time_frameid_->at(frame_count_l);
            auto j_tid = int_frameid2_time_frameid_->at(frame_count_r);

            if (landmark.second.obs.find(i_tid) != landmark.second.obs.end() && landmark.second.obs.find(j_tid) != landmark.second.obs.end())
            {

                Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();

                a = landmark.second.obs.at(i_tid).point;
                b = landmark.second.obs.at(j_tid).point;

                corres[c].push_back(make_pair(a, b));
            }
        }
    }
    return corres;
}
Eigen::aligned_vector<Eigen::aligned_vector<pair<Vector3d, Vector3d>>>
FeatureManager::getCorrespondingWithDepth(int frame_count_l, int frame_count_r)
{
    Eigen::aligned_vector<Eigen::aligned_vector<pair<Vector3d, Vector3d>>> corres;
    corres.resize(NUM_OF_CAM);
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        for (const auto &landmark : KeyPointLandmarks[c])
        {
            auto i_tid = int_frameid2_time_frameid_->at(frame_count_l);
            auto j_tid = int_frameid2_time_frameid_->at(frame_count_r);

            if (landmark.second.obs.find(i_tid) != landmark.second.obs.end() && landmark.second.obs.find(j_tid) != landmark.second.obs.end())
            {

                Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();

                double depth_a = landmark.second.obs.at(i_tid).depth_measured;

                if (depth_a < MIN_DEPTH_MEASURED || depth_a > MAX_DEPTH_MEASURED) // max and min measurement
                    continue;
                double depth_b = landmark.second.obs.at(j_tid).depth_measured;
                //  std::cout<<"depth_b:"<<depth_b<<std::endl;
                if (depth_b < MIN_DEPTH_MEASURED || depth_b > MAX_DEPTH_MEASURED) // max and min measurement
                    continue;

                a = landmark.second.obs.at(i_tid).point;
                b = landmark.second.obs.at(j_tid).point;
                a = a * depth_a;
                b = b * depth_b;

                corres[c].push_back(make_pair(a, b));
            }
        }
    }
    return corres;
}

void FeatureManager::setCamState(vector<bool> camState)
{
    cam_state = camState;
}

void FeatureManager::invDepth2Depth()
{
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        for (auto &landmark : KeyPointLandmarks[c])
        {
            landmark.second.used_num = landmark.second.obs.size();
            if (!(landmark.second.used_num >= 2 && time_frameid2_int_frameid_->at(landmark.second.kf_id) < WINDOW_SIZE - 2))
                continue;

            landmark.second.estimated_depth = 1.0 / landmark.second.data[0];
            // ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
            if (landmark.second.estimated_depth < 0)
            {
                landmark.second.solve_flag = KeyPointLandmark::SolveFlag::SOLVE_FAIL;
            }
            else
                landmark.second.solve_flag = KeyPointLandmark::SolveFlag::SOLVE_SUCC;
        }
    }
}
void FeatureManager::depth2InvDepth()
{
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        for (auto &landmark : KeyPointLandmarks[c])
        {
            landmark.second.used_num = landmark.second.obs.size();
            if (!(landmark.second.used_num >= 2 && time_frameid2_int_frameid_->at(landmark.second.kf_id) < WINDOW_SIZE - 2))
                continue;
            landmark.second.data[0] = 1. / landmark.second.estimated_depth;
        }
    }
}
void FeatureManager::resetDepth()
{
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        for (auto &landmark : KeyPointLandmarks[c])
        {
            landmark.second.used_num = landmark.second.obs.size();
            if (!(landmark.second.used_num >= 2 && time_frameid2_int_frameid_->at(landmark.second.kf_id) < WINDOW_SIZE - 2))
                continue;
            landmark.second.estimated_depth = -1.;
        }
    }
}

void FeatureManager::removeFailures()
{
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        for (auto iter = KeyPointLandmarks[c].begin(); iter != KeyPointLandmarks[c].end();)
        {
            if (iter->second.solve_flag == KeyPointLandmark::SolveFlag::SOLVE_FAIL)
            {
                iter = KeyPointLandmarks[c].erase(iter);
                continue;
            }
            ++iter;
        }
    }
}

// https://blog.csdn.net/kokerf/article/details/72844455
void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], int base_cam)
{
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
#if SINGLE_CAM_DEBUG
        if (c != base_cam)
            continue;
#endif
        for (auto &landmark : KeyPointLandmarks[c])
        {
            landmark.second.used_num = landmark.second.obs.size();
            if (!(landmark.second.used_num >= 2 && time_frameid2_int_frameid_->at(landmark.second.kf_id) < WINDOW_SIZE - 2))
                continue;

            if (landmark.second.estimated_depth > 0)
                continue;

            auto host_tid = landmark.second.kf_id;
            auto host_id = time_frameid2_int_frameid_->at(host_tid);


            Eigen::Matrix<double, 3, 4> P0;
            Eigen::Vector3d t0 = Ps[host_id] + Rs[host_id] * tic[c];
            Eigen::Matrix3d R0 = Rs[host_id] * ric[c];
            P0.leftCols<3>() = Eigen::Matrix3d::Identity();
            P0.rightCols<1>() = Eigen::Vector3d::Zero();

            Eigen::MatrixXd svd_A(2 * landmark.second.obs.size(), 4);
            int svd_idx = 0;
            for (const auto &it_per_frame : landmark.second.obs)
            {

                auto target_tid = it_per_frame.first;
                auto target_id = time_frameid2_int_frameid_->at(target_tid);

                Eigen::Vector3d t1 = Ps[target_id] + Rs[target_id] * tic[c];
                Eigen::Matrix3d R1 = Rs[target_id] * ric[c];
                Eigen::Vector3d t = R0.transpose() * (t1 - t0);
                Eigen::Matrix3d R = R0.transpose() * R1;
                Eigen::Matrix<double, 3, 4> P;
                P.leftCols<3>() = R.transpose();
                P.rightCols<1>() = -R.transpose() * t;
                Eigen::Vector3d f = it_per_frame.second.point.normalized();
                svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
                svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);
            }
            ROS_ASSERT(svd_idx == svd_A.rows());
            Eigen::Vector4d
                svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
            double svd_method = svd_V[2] / svd_V[3];
            // it_per_id->estimated_depth = -b / A;
            // it_per_id->estimated_depth = svd_V[2] / svd_V[3];

            landmark.second.estimated_depth = svd_method;
            // it_per_id->estimated_depth = INIT_DEPTH;

            // 0 initial; 1 by depth image; 2 by triangulate
            landmark.second.estimate_flag = KeyPointLandmark::EstimateFlag::TRIANGULATE;
            if (landmark.second.estimated_depth < 0.1)
            {
                landmark.second.estimated_depth = INIT_DEPTH;
                landmark.second.estimate_flag = KeyPointLandmark::EstimateFlag::NOT_INITIAL;
            }
        }
    }
} // triangulate

void FeatureManager::triangulateWithDepth(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], int base_cam)
{
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
#if SINGLE_CAM_DEBUG
        if (c != base_cam)
            continue;
#endif
        for (auto &landmark : KeyPointLandmarks[c])
        {
            landmark.second.used_num = landmark.second.obs.size();

            if (!(landmark.second.used_num >= 2 && time_frameid2_int_frameid_->at(landmark.second.kf_id) < WINDOW_SIZE - 2))
                continue;

            if (landmark.second.estimated_depth > 0)
                continue;


            auto host_tid = landmark.second.kf_id;
            auto host_id = time_frameid2_int_frameid_->at(host_tid);
            // LOG(INFO)<<"host t:"<<host_tid<<"host_id:"<<host_id;
            vector<double> verified_depths;

            Eigen::Vector3d tr = Ps[host_id] + Rs[host_id] * tic[c];
            Eigen::Matrix3d Rr = Rs[host_id] * ric[c];

            // LOG(INFO)<<landmark.second.obs.size();
            for (auto &it_per_frame : landmark.second.obs)
            {
                // LOG(INFO)<<it_per_frame.first<<","<<it_per_frame.second.point;
                auto target_tid = it_per_frame.first;
                // LOG(INFO)<<"target_tid:"<<target_tid<<", times size:"<<time_frameid2_int_frameid_->size();
                // for (auto i : *time_frameid2_int_frameid_)
                // {
                //     LOG(INFO)<<i.first<<","<<i.second;
                // }
                auto target_id = time_frameid2_int_frameid_->at(target_tid);
                // LOG(INFO)<<"target_id:"<<target_id;
                Eigen::Vector3d t0 = Ps[target_id] + Rs[target_id] * tic[c];
                Eigen::Matrix3d R0 = Rs[target_id] * ric[c];
                // double depth_threshold = 3; //for handheld and wheeled application. Since d435i <3 is quiet acc
                // double depth_threshold = 10; //for tracked application, since IMU quite noisy in this scene

                if (it_per_frame.second.depth_measured < MIN_DEPTH_MEASURED || it_per_frame.second.depth_measured > MAX_DEPTH_MEASURED)
                    continue;

                Eigen::Vector3d
                    point0(it_per_frame.second.point * it_per_frame.second.depth_measured);

                // transform to reference frame
                Eigen::Vector3d t2r = Rr.transpose() * (t0 - tr);
                Eigen::Matrix3d R2r = Rr.transpose() * R0;

                for (auto &it_per_frame2 : landmark.second.obs)
                {
                    if (it_per_frame.first == it_per_frame2.first)
                        continue;

                    auto target_tid = it_per_frame2.first;
                    auto target_id = time_frameid2_int_frameid_->at(target_tid);

                    Eigen::Vector3d t1 = Ps[target_id] + Rs[target_id] * tic[c];
                    Eigen::Matrix3d R1 = Rs[target_id] * ric[c];
                    Eigen::Vector3d t20 = R0.transpose() * (t1 - t0);
                    Eigen::Matrix3d R20 = R0.transpose() * R1;

                    Eigen::Vector3d point1_projected = R20.transpose() * point0 - R20.transpose() * t20;
                    Eigen::Vector2d
                        point1_2d(it_per_frame2.second.point.x(), it_per_frame2.second.point.y());
                    Eigen::Vector2d residual = point1_2d - Vector2d(point1_projected.x() / point1_projected.z(),
                                                                    point1_projected.y() / point1_projected.z());
                    if (residual.norm() < 10.0 / 460)
                    { // this can also be adjust to improve performance
                        Eigen::Vector3d point_r = R2r * point0 + t2r;
                        verified_depths.push_back(point_r.z());
                    }
                }
            }

            if (verified_depths.empty())
                continue;
            double depth_sum = std::accumulate(std::begin(verified_depths), std::end(verified_depths), 0.0);
            double depth_ave = depth_sum / verified_depths.size();
            //        for (int i=0;i<(int)verified_depths.size();i++){
            //            cout << verified_depths[i]<<"|";
            //        }
            //        cout << endl;
            landmark.second.estimated_depth = depth_ave;
            landmark.second.estimate_flag = KeyPointLandmark::EstimateFlag::DIRECT_MEASURED;

            if (landmark.second.estimated_depth < 1)
            {
                landmark.second.estimated_depth = INIT_DEPTH;
                landmark.second.estimate_flag = KeyPointLandmark::EstimateFlag::NOT_INITIAL;
            }
        }
    }
} // triangulateWithDepth

void FeatureManager::removeOneFrameObservation(MCVIO::TimeFrameId marg_frame_tid)
{
    // LOG(INFO) << "marg_frame_tid: " << marg_frame_tid;
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        for (auto iter = KeyPointLandmarks[c].begin(); iter != KeyPointLandmarks[c].end();)
        {
            auto &obs = iter->second.obs;
            if (obs.find(marg_frame_tid) != obs.end())
            {

                obs.erase(marg_frame_tid);
                if (obs.empty())
                {
                    iter = KeyPointLandmarks[c].erase(iter);
                    continue;
                }
                iter->second.kf_id = obs.begin()->first; 
                CHECK(iter->second.kf_id != 0) << "update kf_id error";
            }
            ++iter;
        }
    }
}

void FeatureManager::removeOneFrameObservationAndShiftDepth(MCVIO::TimeFrameId marg_frame_tid,
                                                            Vector3d Ps[],
                                                            Vector3d tic[], Matrix3d ric[])
{
    // LOG(INFO) << "marg_frame_tid: " << std::to_string(marg_frame_tid);
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        auto oldhost_tid = marg_frame_tid;
        int oldhost_id = time_frameid2_int_frameid_->at(oldhost_tid);

        Transformd T_i_c(ric[c], tic[c]);
        Transformd T_c_i = T_i_c.inverse();
        Transformd T_w_oldhost_i(Rs[oldhost_id], Ps[oldhost_id]);

        for (auto iter = KeyPointLandmarks[c].begin(); iter != KeyPointLandmarks[c].end();)
        {
            auto &obs = iter->second.obs;
            if (obs.find(marg_frame_tid) != obs.end())
            {

                CHECK(iter->second.obs.begin()->first == marg_frame_tid) << "ERROR";
                Eigen::Vector3d normalized_point = iter->second.obs.begin()->second.point;
                Eigen::Vector3d pts_oldhost = normalized_point * iter->second.estimated_depth;

                obs.erase(marg_frame_tid);
                if (obs.empty())
                {
                    iter = KeyPointLandmarks[c].erase(iter);
                    continue;
                }

                if (iter->second.kf_id != obs.begin()->first)
                {

                    iter->second.kf_id = obs.begin()->first; 

                    auto newhost_tid = iter->second.kf_id;

                    int newhost_id = time_frameid2_int_frameid_->at(newhost_tid);

                    Transformd T_w_newhost_i(Rs[newhost_id], Ps[newhost_id]);
                    Transformd T_newhost_c_oldhost_c = T_c_i * T_w_newhost_i.inverse() * T_w_oldhost_i * T_i_c;

                    Eigen::Vector3d pts_newhost = T_newhost_c_oldhost_c * pts_oldhost;
                    double newdepth = pts_newhost(2);
                    if (newdepth > 0)
                        iter->second.estimated_depth = newdepth;
                    else
                        iter->second.estimated_depth = INIT_DEPTH;
                }
            }
            ++iter;
        }
    }
} // function removeOneFrameObservationAndShiftDepth

double
FeatureManager::compensatedParallax2(const KeyPointLandmark &it_per_id, int frame_count)
{
    // check the second last frame is keyframe or not
    // parallax betwwen seconde last frame and third last frame

    if (frame_count < 1)
        return 0;
    auto target1_tid = int_frameid2_time_frameid_->at(frame_count - 2);
    auto target2_tid = int_frameid2_time_frameid_->at(frame_count - 1);

    CHECK(it_per_id.obs.find(target1_tid) != it_per_id.obs.end() &&
          it_per_id.obs.find(target2_tid) != it_per_id.obs.end())
        << "newest two frame don't have fature observaton";

    const auto &target1_obs = it_per_id.obs.at(target1_tid);
    const auto &target2_obs = it_per_id.obs.at(target2_tid);

    double ans = 0;
    Vector3d p_j = target2_obs.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = target1_obs.point;
    Vector3d p_i_comp;

    // int r_i = frame_count - 2;
    // int r_j = frame_count - 1;
    // p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}

#if 1
bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P,
                                    vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    // printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    // pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if (!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    // cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}

void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[], int base_cam)
{

    if (frameCnt > 0)
    {
        vector<cv::Point2f> pts2D;
        vector<cv::Point3f> pts3D;
        // for (auto &it_per_id : feature)
        // {
        //     if (it_per_id.estimated_depth > 0)
        //     {
        //         int index = frameCnt - it_per_id.start_frame;
        //         if((int)it_per_id.feature_per_frame.size() >= index + 1)
        //         {
        //             Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];
        //             Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];

        //             cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
        //             cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());
        //             pts3D.push_back(point3d);
        //             pts2D.push_back(point2d);
        //         }
        //     }
        // }
        for (auto &landmark : KeyPointLandmarks[base_cam])
        {
            if (landmark.second.measured_depth > MIN_DEPTH_MEASURED &&
                landmark.second.measured_depth < MAX_DEPTH_MEASURED)
            {
                auto host_tid = landmark.second.kf_id;
                auto host_id = time_frameid2_int_frameid_->at(host_tid);
                int index = frameCnt - host_id;
                // LOG(INFO)<<"Index:"<<index<<'\n'<<"host_id:"<<host_id;
                // LOG(INFO)<<"Size:"<<(int)landmark.second.obs.size() ;
                if ((int)landmark.second.obs.size() >= index + 1)
                {
                    Vector3d ptsInCam = ric[base_cam] * (landmark.second.obs.at(host_tid).point * landmark.second.obs.at(host_tid).depth_measured) + tic[base_cam];
                    Vector3d ptsInWorld = Rs[host_id] * ptsInCam + Ps[host_id];

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());

                    auto it = landmark.second.obs.begin();
                    std::advance(it, index);
                    // cv::Point2f point2d(landmark.second.obs[index].point.x(), landmark.second.obs[index].point.y());
                    cv::Point2f point2d(it->second.point.x(), it->second.point.y());
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d);
                }
            }
        }
        LOG(INFO) << "Pts num:" << pts3D.size();
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // trans to w_T_cam
        RCam = Rs[frameCnt - 1] * ric[base_cam];
        PCam = Rs[frameCnt - 1] * tic[base_cam] + Ps[frameCnt - 1];

        if (solvePoseByPnP(RCam, PCam, pts2D, pts3D))
        {
            // trans to w_T_imu
            Rs[frameCnt] = RCam * ric[base_cam].transpose();
            Ps[frameCnt] = -RCam * ric[base_cam].transpose() * tic[base_cam] + PCam;

            Eigen::Quaterniond Q(Rs[frameCnt]);

            LOG(INFO) << "init Frame by PnP success";
            // cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            // cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
        }
    }
}
#endif