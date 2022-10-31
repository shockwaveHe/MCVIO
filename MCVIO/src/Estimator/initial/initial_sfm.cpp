#include "initial_sfm.h"

GlobalSFM::GlobalSFM()
{
    feature_num.resize(NUM_OF_CAM);
}

void GlobalSFM::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                                 Eigen::Matrix<double, 3, 4> &Pose1,
                                 Vector2d &point0, Vector2d &point1,
                                 Vector3d &point_3d)
{
    Matrix4d design_matrix = Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Vector4d triangulated_point;
    triangulated_point =
        design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

bool GlobalSFM::solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i,
                                Eigen::aligned_vector<Eigen::aligned_vector<SFMFeature>> &sfm_f, int base_cam)
{
    vector<cv::Point2f> pts_2_vector;
    vector<cv::Point3f> pts_3_vector;

    int num_state = 0;

    for (int j = 0; j < feature_num[base_cam]; j++)
    {
        if (sfm_f[base_cam][j].state != true)
        {
            num_state = num_state + 1;

            continue;
        }
        Vector2d point2d;
        
        for (int k = 0; k < (int)sfm_f[base_cam][j].observation.size(); k++)
        {
          
            if (sfm_f[base_cam][j].observation[k].first == i)
            {
                
                Vector2d img_pts = sfm_f[base_cam][j].observation[k].second;
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);

                
                cv::Point3f pts_3(sfm_f[base_cam][j].position[0],
                                  sfm_f[base_cam][j].position[1],
                                  sfm_f[base_cam][j].position[2]);
                pts_3_vector.push_back(pts_3);
                break;
            }
        }
    }
    LOG(INFO) << " feature_num: " << feature_num[base_cam] << " state_num: " << num_state;

    LOG(INFO) << "int(pts_2_vector.size()): " << int(pts_2_vector.size());
    if (int(pts_2_vector.size()) < 15) 
    {
        printf("unstable features tracking, please slowly move you device!\n");
        if (int(pts_2_vector.size()) < 8) // VINS-mono threshold is 10, but 6 can give a solution
                                          // TODO: For keypoints less than 8, use IMU to predict pose since with PnP the scale is close to 1
            return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    
    // TODO: Discuss whether to change K
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool pnp_succ;
    
    pnp_succ = cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1);
    if (!pnp_succ)
    {
        LOG(INFO) << "Opencv pnp failure!";
        return false;
    }
    cv::Rodrigues(rvec, r);
    MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    R_initial = R_pnp;
    P_initial = T_pnp;
    return true;
}

void GlobalSFM::triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
                                     int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
                                     Eigen::aligned_vector<Eigen::aligned_vector<SFMFeature>> &sfm_f, int base_cam)
{
    // LOG(INFO) << "NOT USING DETPH TO finish initialization! ";
    assert(frame0 != frame1);
    for (int j = 0; j < feature_num[base_cam]; j++)
    {
        if (sfm_f[base_cam][j].state == true)
            continue;
        bool has_0 = false, has_1 = false;
        Vector2d point0;
        Vector2d point1;
        for (int k = 0; k < (int)sfm_f[base_cam][j].observation.size(); k++)
        {
            if (sfm_f[base_cam][j].observation[k].first == frame0)
            {
                point0 = sfm_f[base_cam][j].observation[k].second;
                has_0 = true;
            }
            if (sfm_f[base_cam][j].observation[k].first == frame1)
            {
                point1 = sfm_f[base_cam][j].observation[k].second;
                has_1 = true;
            }
        }
        if (has_0 && has_1)
        {
            Vector3d point_3d;
            triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
            sfm_f[base_cam][j].state = true;
            sfm_f[base_cam][j].position[0] = point_3d(0);
            sfm_f[base_cam][j].position[1] = point_3d(1);
            sfm_f[base_cam][j].position[2] = point_3d(2);
            // cout << "trangulated : " << frame1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
        }
    }
}

// Triangulate points depth
void GlobalSFM::triangulateTwoFramesWithDepth(int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
                                              int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
                                              Eigen::aligned_vector<Eigen::aligned_vector<SFMFeature>> &sfm_f, int base_cam)
{
    LOG(INFO) << "Use Depth Information to finish initialization! ";
    assert(frame0 != frame1);
    Matrix3d Pose0_R = Pose0.block<3, 3>(0, 0);
    Matrix3d Pose1_R = Pose1.block<3, 3>(0, 0);
    Vector3d Pose0_t = Pose0.block<3, 1>(0, 3);
    Vector3d Pose1_t = Pose1.block<3, 1>(0, 3);

    for (int j = 0; j < feature_num[base_cam]; j++)
    {
        if (sfm_f[base_cam][j].state)
            continue;
        bool has_0 = false, has_1 = false;
        bool depth0_valid = true;
        bool depth1_valid = true;
        Vector3d point0;
        Vector3d point1;

        Vector2d point00;
        Vector2d point01;

        for (int k = 0; k < (int)sfm_f[base_cam][j].observation.size(); k++)
        {

            if (sfm_f[base_cam][j].observation[k].first == frame0)
            {
                if (sfm_f[base_cam][j].observation_depth[k].second < 1 || sfm_f[base_cam][j].observation_depth[k].second > 100)
                    depth0_valid = false;
                if (depth0_valid)
                {
                    point0 = Vector3d(sfm_f[base_cam][j].observation[k].second.x() * sfm_f[base_cam][j].observation_depth[k].second,
                                      sfm_f[base_cam][j].observation[k].second.y() * sfm_f[base_cam][j].observation_depth[k].second,
                                      sfm_f[base_cam][j].observation_depth[k].second);
                }
                point00 = sfm_f[base_cam][j].observation[k].second;
                has_0 = true;
            }

            if (sfm_f[base_cam][j].observation[k].first == frame1)
            {

                if (sfm_f[base_cam][j].observation_depth[k].second < 1 || sfm_f[base_cam][j].observation_depth[k].second > 100)
                    depth1_valid = false;
                if (depth1_valid)
                {
                    point1 = Vector3d(sfm_f[base_cam][j].observation[k].second.x() * sfm_f[base_cam][j].observation_depth[k].second,
                                      sfm_f[base_cam][j].observation[k].second.y() * sfm_f[base_cam][j].observation_depth[k].second,
                                      sfm_f[base_cam][j].observation_depth[k].second);
                }
                point01 = sfm_f[base_cam][j].observation[k].second;
                has_1 = true;
            }
        }

        if (has_0 && has_1 && depth0_valid && depth1_valid)
        {
            Vector2d residual;
            Vector3d point_3d, point1_reprojected;
            // triangulatePoint(Pose0, Pose1, point0, point1, point_3d);

            //  P_l_l+1=R_l+1_l.inverse()*P - R_l+1_l.inverse()*T_l+1_l
            point_3d = Pose0_R.transpose() * point0 - Pose0_R.transpose() * Pose0_t; // shan add:this is point in world;
            // P_cur_l+1=R_cur_l* P_l_l+1+T_cur_l
            point1_reprojected = Pose1_R * point_3d + Pose1_t;


            residual = point01 - Vector2d(point1_reprojected.x() / point1_reprojected.z(),
                                          point1_reprojected.y() / point1_reprojected.z());

            // std::cout << residual.transpose()<<"norm"<<residual.norm()*460<<endl;

            if (residual.norm() < 1.0 / 460)
            {
                sfm_f[base_cam][j].state = true;

                sfm_f[base_cam][j].position[0] = point_3d(0);
                sfm_f[base_cam][j].position[1] = point_3d(1);
                sfm_f[base_cam][j].position[2] = point_3d(2);
            }
            // cout << "trangulated : " << frame1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
        }
        else if (has_0 && has_1 && depth0_valid)
        {
            Vector3d point_3d;

            point_3d = Pose0_R.transpose() * point0 - Pose0_R.transpose() * Pose0_t; 
            point_3d = Pose1_R * point_3d + Pose1_t;

            Vector2d residual;
            residual = point01 - Vector2d(point_3d.x() / point_3d.z(),
                                          point_3d.y() / point_3d.z());

            if (residual.norm() < 1.0 / 460)
            {
                sfm_f[base_cam][j].state = true;

                sfm_f[base_cam][j].position[0] = point_3d(0);
                sfm_f[base_cam][j].position[1] = point_3d(1);
                sfm_f[base_cam][j].position[2] = point_3d(2);
            }
        }
        else if (has_0 && has_1 && depth1_valid)
        {
            Vector3d point_3d, point1_reprojected;

            point_3d = Pose1_R.transpose() * point1 - Pose1_R.transpose() * Pose1_t; 
            point_3d = Pose0_R * point_3d + Pose0_t;

            Vector2d residual;
            residual = point00 - Vector2d(point_3d.x() / point_3d.z(),
                                          point_3d.y() / point_3d.z());

            if (residual.norm() < 1.0 / 460)
            {
                sfm_f[base_cam][j].state = true;

                sfm_f[base_cam][j].position[0] = point_3d(0);
                sfm_f[base_cam][j].position[1] = point_3d(1);
                sfm_f[base_cam][j].position[2] = point_3d(2);
            }
        }
        if (has_0 && has_1 && !sfm_f[base_cam][j].state)
        {
            Vector3d point_3d;
            triangulatePoint(Pose0, Pose1, point00, point01, point_3d);
            sfm_f[base_cam][j].state = true;
            sfm_f[base_cam][j].position[0] = point_3d(0);
            sfm_f[base_cam][j].position[1] = point_3d(1);
            sfm_f[base_cam][j].position[2] = point_3d(2);
        }
    }
}

// 	 q w_R_cam t w_R_cam
//  c_rotation cam_R_w
//  c_translation cam_R_w
// relative_q[i][j]  j_q_i
// relative_t[i][j]  j_t_ji  (j < i)

bool GlobalSFM::construct(int frame_num,
                          Eigen::aligned_vector<Eigen::aligned_vector<Quaterniond>> &q,
                          Eigen::aligned_vector<Eigen::aligned_vector<Vector3d>> &T,
                          int l,
                          int base_cam,
                          const Matrix3d relative_R,
                          const Vector3d relative_T,
                          Eigen::aligned_vector<Eigen::aligned_vector<SFMFeature>> &sfm_f,
                          Eigen::aligned_vector<Eigen::aligned_map<int, Vector3d>> &sfm_tracked_points,
                          Matrix3d ric[],
                          Vector3d tic[])
{   
    LOG(INFO)<<"Construct";
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        feature_num[c] = sfm_f[c].size();
        LOG(INFO)<<"Feature num "<< c << ": "<<feature_num[c];
    }
    
    // cout << "set 0 and " << l << " as known " << endl;
    //  have relative_r relative_t
    //  intial two view

    q[base_cam][l].w() = 1;
    q[base_cam][l].x() = 0;
    q[base_cam][l].y() = 0;
    q[base_cam][l].z() = 0;
    T[base_cam][l].setZero();
    q[base_cam][frame_num - 1] = q[base_cam][l] * Quaterniond(relative_R);
    T[base_cam][frame_num - 1] = relative_T;
    // cout << "init q_l " << q[l].w() << " " << q[l].vec().transpose() << endl;
    // cout << "init t_l " << T[l].transpose() << endl;

    // rotate to cam frame
    Matrix3d c_Rotation[NUM_OF_CAM][frame_num];
    Vector3d c_Translation[NUM_OF_CAM][frame_num];
    Quaterniond c_Quat[NUM_OF_CAM][frame_num];
    double c_rotation[NUM_OF_CAM][frame_num][4];
    double c_translation[frame_num][NUM_OF_CAM][3];
    Eigen::Matrix<double, 3, 4> Pose[NUM_OF_CAM][frame_num];


    c_Quat[base_cam][l] = q[base_cam][l].inverse();
    c_Rotation[base_cam][l] = c_Quat[base_cam][l].toRotationMatrix();
    c_Translation[base_cam][l] = -1 * (c_Rotation[base_cam][l] * T[base_cam][l]);
    Pose[base_cam][l].block<3, 3>(0, 0) = c_Rotation[base_cam][l];
    Pose[base_cam][l].block<3, 1>(0, 3) = c_Translation[base_cam][l];

    c_Quat[base_cam][frame_num - 1] = q[base_cam][frame_num - 1].inverse();
    c_Rotation[base_cam][frame_num - 1] = c_Quat[base_cam][frame_num - 1].toRotationMatrix();
    c_Translation[base_cam][frame_num - 1] = -1 * (c_Rotation[base_cam][frame_num - 1] * T[base_cam][frame_num - 1]);
    Pose[base_cam][frame_num - 1].block<3, 3>(0, 0) = c_Rotation[base_cam][frame_num - 1];
    Pose[base_cam][frame_num - 1].block<3, 1>(0, 3) = c_Translation[base_cam][frame_num - 1];

    // 1: trangulate between l ----- frame_num - 1
    // 2: solve pnp l + 1; trangulate l + 1 ------- frame_num - 1;

    for (int i = l; i < frame_num - 1; i++)
    {
        // solve pnp
        if (i > l)
        {
            Matrix3d R_initial = c_Rotation[base_cam][i - 1];
            Vector3d P_initial = c_Translation[base_cam][i - 1];
            if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f, base_cam))
            {
                LOG(INFO) << "1 solveFrameByPnP  failure";
                return false;
            }
            c_Rotation[base_cam][i] = R_initial;
            c_Translation[base_cam][i] = P_initial;
            c_Quat[base_cam][i] = c_Rotation[base_cam][i];
            Pose[base_cam][i].block<3, 3>(0, 0) = c_Rotation[base_cam][i];
            Pose[base_cam][i].block<3, 1>(0, 3) = c_Translation[base_cam][i];
        }
        // triangulate point based on the solve pnp result
        // keypoint triangulation for points in base_cam
        if (USE_DEPTH_INIT)
            triangulateTwoFramesWithDepth(i, Pose[base_cam][i], frame_num - 1, Pose[base_cam][frame_num - 1], sfm_f, base_cam);
        else
            triangulateTwoFrames(i, Pose[base_cam][i], frame_num - 1, Pose[base_cam][frame_num - 1], sfm_f, base_cam);
    }
    // 3: triangulate l-----l+1 l+2 ... frame_num -2
    for (int i = l + 1; i < frame_num - 1; i++)

        if (USE_DEPTH_INIT)
            triangulateTwoFramesWithDepth(l, Pose[base_cam][l], i, Pose[base_cam][i], sfm_f, base_cam);
        else
            triangulateTwoFrames(l, Pose[base_cam][l], i, Pose[base_cam][i], sfm_f, base_cam);
    // 4: solve pnp l-1; triangulate l-1 ----- l
    //              l-2              l-2 ----- l

    for (int i = l - 1; i >= 0; i--)
    {
        // solve pnp
        Matrix3d R_initial = c_Rotation[base_cam][i + 1];
        Vector3d P_initial = c_Translation[base_cam][i + 1];
        if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f, base_cam))
        {
            LOG(INFO) << "current frame:" << i << "target frame: " << l;
            LOG(INFO) << "2 solveFrameByPnP  failure";
            return false;
        }
        c_Rotation[base_cam][i] = R_initial;
        c_Translation[base_cam][i] = P_initial;
        c_Quat[base_cam][i] = c_Rotation[base_cam][i];
        Pose[base_cam][i].block<3, 3>(0, 0) = c_Rotation[base_cam][i];
        Pose[base_cam][i].block<3, 1>(0, 3) = c_Translation[base_cam][i];
        // triangulate

        if (USE_DEPTH_INIT)
            triangulateTwoFramesWithDepth(i, Pose[base_cam][i], l, Pose[base_cam][l], sfm_f, base_cam);
        else
            triangulateTwoFrames(i, Pose[base_cam][i], l, Pose[base_cam][l], sfm_f, base_cam);
    }
    // 5: triangulate all other points
    for (int j = 0; j < feature_num[base_cam]; j++)
    {
        if (sfm_f[base_cam][j].state == true)
            continue;
        if ((int)sfm_f[base_cam][j].observation.size() >= 2)
        {

            Vector2d point0, point1;
            int frame_0 = sfm_f[base_cam][j].observation[0].first;
            point0 = sfm_f[base_cam][j].observation[0].second;
            int frame_1 = sfm_f[base_cam][j].observation.back().first;
            point1 = sfm_f[base_cam][j].observation.back().second;
            Vector3d point_3d;
            triangulatePoint(Pose[base_cam][frame_0], Pose[base_cam][frame_1], point0, point1, point_3d);
            sfm_f[base_cam][j].state = true;
            sfm_f[base_cam][j].position[0] = point_3d(0);
            sfm_f[base_cam][j].position[1] = point_3d(1);
            sfm_f[base_cam][j].position[2] = point_3d(2);
            // cout << "trangulated : " << frame_0 << " " << frame_1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
        }
    }

    // full BA

    ceres::Problem problem;
    ceres::LocalParameterization *local_parameterization = new ceres::QuaternionParameterization();
    // cout << " begin full BA " << endl;
    for (int i = 0; i < frame_num; i++)
    {
        // double array for ceres
        c_translation[base_cam][i][0] = c_Translation[base_cam][i].x();
        c_translation[base_cam][i][1] = c_Translation[base_cam][i].y();
        c_translation[base_cam][i][2] = c_Translation[base_cam][i].z();
        c_rotation[base_cam][i][0] = c_Quat[base_cam][i].w();
        c_rotation[base_cam][i][1] = c_Quat[base_cam][i].x();
        c_rotation[base_cam][i][2] = c_Quat[base_cam][i].y();
        c_rotation[base_cam][i][3] = c_Quat[base_cam][i].z();
        problem.AddParameterBlock(c_rotation[base_cam][i], 4, local_parameterization);
        problem.AddParameterBlock(c_translation[base_cam][i], 3);
        if (i == l)
        {
            problem.SetParameterBlockConstant(c_rotation[base_cam][i]);
        }
        if (i == l || i == frame_num - 1)
        {
            problem.SetParameterBlockConstant(c_translation[base_cam][i]);
        }
    }

    for (int i = 0; i < feature_num[base_cam]; i++)
    {
        if (sfm_f[base_cam][i].state != true)
            continue;
        for (int j = 0; j < int(sfm_f[base_cam][i].observation.size()); j++)
        {
            int l = sfm_f[base_cam][i].observation[j].first;
            ceres::CostFunction *cost_function = ReprojectionError3D::Create(
                sfm_f[base_cam][i].observation[j].second.x(),
                sfm_f[base_cam][i].observation[j].second.y());

            problem.AddResidualBlock(cost_function, nullptr, c_rotation[base_cam][l], c_translation[base_cam][l],
                                     sfm_f[base_cam][i].position);
            //
            // problem.SetParameterBlockConstant(sfm_f[i].position);?
            // No: sfm_f[i].position is obtained through estimated pose + true depth
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.2;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << "\n";
    if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03)
    {
        cout << "vision only BA converge" << endl;
    }
    else
    {
        cout << "vision only BA not converge " << endl;
        return false;
    }
    for (int i = 0; i < frame_num; i++)
    {
        q[base_cam][i].w() = c_rotation[base_cam][i][0];
        q[base_cam][i].x() = c_rotation[base_cam][i][1];
        q[base_cam][i].y() = c_rotation[base_cam][i][2];
        q[base_cam][i].z() = c_rotation[base_cam][i][3];
        q[base_cam][i] = q[base_cam][i].inverse();
        // cout << "final  q" << " i " << i <<"  " <<q[i].w() << "  " << q[i].vec().transpose() << endl;
    }
    for (int i = 0; i < frame_num; i++)
    {
        // T' = T^-1
        T[base_cam][i] = -1 * (q[base_cam][i] * Vector3d(c_translation[base_cam][i][0], c_translation[base_cam][i][1], c_translation[base_cam][i][2]));
        LOG(INFO) << "final  t"
                  << " i " << i << "  " << T[base_cam][i](0) << "  " << T[base_cam][i](1) << "  " << T[base_cam][i](2);
    }

    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        if (c == base_cam)
            continue;
#if SINGLE_CAM_DEBUG
        if (c != base_cam)
            continue;
#endif
        for (int i = 0; i < frame_num; i++)
        {
            // required double check
            q[c][i] = q[base_cam][i] * Quaterniond(ric[base_cam].transpose()) * Quaterniond(ric[c]);
            // T[c][i] = ric[c] * (ric[base_cam].transpose() * T[base_cam][i] - ric[base_cam].transpose() * tic[base_cam]) + tic[c];
            // cout<<"previous:"<< T[c][i]<<endl;
            // T[c][i] = (q[base_cam][i].toRotationMatrix()*ric[base_cam].transpose())*(tic[c]-tic[base_cam])+T[base_cam][i];
            T[c][i] = T[base_cam][i];
            // cout<<"Now:"<< T[c][i]<<endl;
            // cout<<"------"<<endl;
            c_Quat[c][i] = q[c][i].inverse();
            c_Rotation[c][i] = c_Quat[c][i].toRotationMatrix();
            c_Translation[c][i] = -1 * (c_Rotation[c][i] * T[c][i]);
            Pose[c][i].block<3, 3>(0, 0) = c_Rotation[c][i];
            Pose[c][i].block<3, 1>(0, 3) = c_Translation[c][i];
        }
        // printf("------------------------------------------------------------------------------------------\n File: \"%s\", line: %d, function <%s>\n ======================================================\n", __FILE__, __LINE__, __func__);
        printf("Initial Positions \n");

        // calculated the 3D position of points in other cameras
        for (int c = 0; c < NUM_OF_CAM; c++)
        {
            if (c == base_cam)
                continue;
#if SINGLE_CAM_DEBUG
            if (c != base_cam)
                continue;
#endif
            for (int j = 0; j < feature_num[c]; j++)
            {
                if (sfm_f[c][j].state == true)
                    continue;
                // if (sfm_f[c][j].id(0) != c)
                // 	continue;
                if ((int)sfm_f[c][j].observation.size() >= 2)
                {
                    Vector2d point0, point1;
                    int frame_0 = sfm_f[c][j].observation[0].first;
                    point0 = sfm_f[c][j].observation[0].second;
                    int frame_1 = sfm_f[c][j].observation.back().first;
                    point1 = sfm_f[c][j].observation.back().second;
                    Vector3d point_3d;
                    triangulatePoint(Pose[c][frame_0], Pose[c][frame_1], point0, point1, point_3d);
                    sfm_f[c][j].state = true;
                    sfm_f[c][j].position[0] = point_3d(0);
                    sfm_f[c][j].position[1] = point_3d(1);
                    sfm_f[c][j].position[2] = point_3d(2);
                    // cout << "trangulated : " << frame_0 << " " << frame_1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
                }
            }
        }
    }
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
#if SINGLE_CAM_DEBUG
        if (c != base_cam)
            continue;
#endif
        for (int i = 0; i < (int)sfm_f[c].size(); i++)
        {
            if (sfm_f[c][i].state)
                sfm_tracked_points[c][sfm_f[c][i].id] =
                    Vector3d(sfm_f[c][i].position[0], sfm_f[c][i].position[1], sfm_f[c][i].position[2]);
        }
    }
    return true;
}
