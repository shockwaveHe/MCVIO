#include "visualization.h"

using namespace ros;
using namespace Eigen;

ros::Publisher pub_odometry, pub_latest_odometry, pub_loam_odometry;

ros::Publisher pub_path, pub_relo_path;

ros::Publisher pub_point_cloud, pub_margin_cloud;

ros::Publisher pub_lidar_cloud;

ros::Publisher pub_key_poses;

ros::Publisher pub_relo_relative_pose;

ros::Publisher pub_camera_pose;

// publisher current cam
// ros::Publisher pub_camera_pose_visual;

// publisher slide window camera pose

// ros::Publisher pub_slidewindow_camera_pose;

nav_msgs::Path path, relo_path;

ros::Publisher pub_map_projection_image;

ros::Publisher pub_keyframe_pose;

ros::Publisher pub_keyframe_point;

ros::Publisher pub_extrinsic;

ros::Publisher pub_delta_v;

ros::Publisher pub_delta_p;

ros::Publisher pub_delta_t;

ros::Publisher pub_vloamPath;

nav_msgs::Path violoamPath;

// CameraPoseVisualization cameraposevisual(0, 1, 0, 1);

// CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);

static double sum_of_path = 0;

static Vector3d last_path(0.0, 0.0, 0.0);

vector<int> prev_pcd_id;

double pubOdometry_current_time = -1;

void registerPub(ros::NodeHandle &n)
{
  pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
  pub_path = n.advertise<nav_msgs::Path>("path", 1000);
  pub_relo_path = n.advertise<nav_msgs::Path>("relocalization_path", 1000);
  pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
  pub_loam_odometry = n.advertise<nav_msgs::Odometry>("lidar/odometry", 1000);
  pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
  pub_margin_cloud =
      n.advertise<sensor_msgs::PointCloud>("history_cloud", 1000);
  pub_lidar_cloud = n.advertise<sensor_msgs::PointCloud2>("lidar_cloud", 1000);

  pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
  pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
  pub_map_projection_image =
      n.advertise<sensor_msgs::Image>("depth_image", 1000);
  // pub_camera_pose_visual =
  // n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
  // pub_slidewindow_camera_pose =
  //     n.advertise<visualization_msgs::MarkerArray>("slidewindow_pose", 1000);
  pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
  pub_keyframe_point =
      n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
  pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
  pub_relo_relative_pose =
      n.advertise<nav_msgs::Odometry>("relo_relative_pose", 1000);
  pub_delta_p = n.advertise<std_msgs::Float64>("delta_p", 1000);
  pub_delta_v = n.advertise<std_msgs::Float64>("delta_v", 1000);
  pub_delta_t = n.advertise<std_msgs::Float64>("delta_t", 1000);
  pub_vloamPath = n.advertise<nav_msgs::Path>("vio_loam_path", 100);

  // cameraposevisual.setScale(1);
  // cameraposevisual.setLineWidth(0.05);
  // keyframebasevisual.setScale(1);
  // keyframebasevisual.setLineWidth(0.05);
}

void pubLatestOdometry(const MCVIOEstimator &estimator, const Eigen::Vector3d &P,
                       const Eigen::Quaterniond &Q, const Eigen::Vector3d &V,
                       const std_msgs::Header &header)
{
  Eigen::Quaterniond quadrotor_Q = Q;

  static tf::TransformBroadcaster br;
  static tf::TransformListener listener;
  tf::Transform transform;
  tf::Quaternion q;
  // body frame

  Transformd T_w_i(Q, P);
  Transformd T_i_c(estimator.ric[0], estimator.tic[0]);
  Transformd T_w_l = T_w_i * T_i_c * Tcam_lidar;

  Vector3d correct_t = T_w_l.pos;
  Quaterniond correct_q = T_w_l.rot;
  transform.setOrigin(tf::Vector3(correct_t(0), correct_t(1), correct_t(2)));
  q.setW(correct_q.w());
  q.setX(correct_q.x());
  q.setY(correct_q.y());
  q.setZ(correct_q.z());
  transform.setRotation(q);
  // br.sendTransform(
  //     tf::StampedTransform(transform, header.stamp, "world", "lidar"));

  nav_msgs::Odometry odometry;
  odometry.pose.covariance[0] = double(estimator.failure_occur); // notify lidar odometry failure
  odometry.pose.covariance[1] = double(estimator.initial_stat);  // notify lidar odometry VIO initial stat
  odometry.header = header;
  odometry.header.frame_id = LIO_World_Frame;
  odometry.pose.pose.position.x = T_w_l.pos.x();
  odometry.pose.pose.position.y = T_w_l.pos.y();
  odometry.pose.pose.position.z = T_w_l.pos.z();
  odometry.pose.pose.orientation.x = T_w_l.rot.x();
  odometry.pose.pose.orientation.y = T_w_l.rot.y();
  odometry.pose.pose.orientation.z = T_w_l.rot.z();
  odometry.pose.pose.orientation.w = T_w_l.rot.w();
  odometry.twist.twist.linear.x = V.x();
  odometry.twist.twist.linear.y = V.y();
  odometry.twist.twist.linear.z = V.z();
  pub_latest_odometry.publish(odometry);

  static double last_align_time = -1;

  if (ALIGN_CAMERA_LIDAR_COORDINATE == 1)
  {
    static tf::Transform t_odom_world = tf::Transform(tf::createQuaternionFromRPY(0, 0, M_PI), tf::Vector3(0, 0, 0));
    if (header.stamp.toSec() - last_align_time > 1.0)
    {
      try
      {
        tf::StampedTransform trans_odom_baselink;
        listener.lookupTransform(LIO_World_Frame, LIO_Laser_Frame, ros::Time(0), trans_odom_baselink);
        t_odom_world = transformConversion(trans_odom_baselink) * transform.inverse();
        last_align_time = header.stamp.toSec();
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("No LIO odom tf when align VIO trajectory");
      }
    }
    br.sendTransform(tf::StampedTransform(t_odom_world, header.stamp, LIO_World_Frame, VINS_World_Frame));
  }
  else
  {
    tf::Transform t_static = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
    br.sendTransform(tf::StampedTransform(t_static, header.stamp, LIO_World_Frame, VINS_World_Frame));
  }
}

tf::Transform transformConversion(const tf::StampedTransform &t)
{
  double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
  xCur = t.getOrigin().x();
  yCur = t.getOrigin().y();
  zCur = t.getOrigin().z();
  tf::Matrix3x3 m(t.getRotation());
  m.getRPY(rollCur, pitchCur, yawCur);
  return tf::Transform(tf::createQuaternionFromRPY(rollCur, pitchCur, yawCur), tf::Vector3(xCur, yCur, zCur));
  ;
}

void printStatistics(const MCVIOEstimator &estimator, double t)
{
  if (estimator.solver_flag != MCVIOEstimator::SolverFlag::NON_LINEAR)
    return;
  printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(),
         estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
  ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
  ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
  for (int i = 0; i < NUM_OF_CAM; i++)
  {
    // ROS_DEBUG("calibration result for camera %d", i);
    ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
    ROS_DEBUG_STREAM(
        "extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
    if (ESTIMATE_EXTRINSIC)
    {
      cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
      Eigen::Matrix3d eigen_R;
      Eigen::Vector3d eigen_T;
      eigen_R = estimator.ric[i];
      eigen_T = estimator.tic[i];
      cv::Mat cv_R, cv_T;
      cv::eigen2cv(eigen_R, cv_R);
      cv::eigen2cv(eigen_T, cv_T);
      fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
      fs.release();
    }
  }

  static double sum_of_time = 0;
  static int sum_of_calculation = 0;
  sum_of_time += t;
  sum_of_calculation++;
  ROS_DEBUG("vo solver costs: %f ms", t);
  ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

  sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
  last_path = estimator.Ps[WINDOW_SIZE];
  ROS_DEBUG("sum of path %f", sum_of_path);
  //    if (ESTIMATE_TD)
  //        ROS_INFO("td %f", estimator.td);
}

void pubOdometry(const MCVIOEstimator &estimator, const std_msgs::Header &header)
{
  if (pubOdometry_current_time < 0)
  {
    pubOdometry_current_time = header.stamp.toSec();
  }
  double dt = header.stamp.toSec() - pubOdometry_current_time;
  pubOdometry_current_time = header.stamp.toSec();

  if (estimator.solver_flag == MCVIOEstimator::SolverFlag::NON_LINEAR)
  {
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = VINS_World_Frame;
    odometry.child_frame_id = Camera_Frame;
    Quaterniond tmp_Q;
    tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
    odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
    odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
    odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
    odometry.pose.pose.orientation.x = tmp_Q.x();
    odometry.pose.pose.orientation.y = tmp_Q.y();
    odometry.pose.pose.orientation.z = tmp_Q.z();
    odometry.pose.pose.orientation.w = tmp_Q.w();
    odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
    odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
    odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
    //"odometry"
    pub_odometry.publish(odometry);
    LOG(INFO) << "Latest pose:(" << odometry.pose.pose.position.x << "," << odometry.pose.pose.position.y << "," << odometry.pose.pose.position.z << ")";

    Transformd T_w_i(tmp_Q, estimator.Ps[WINDOW_SIZE]);
    Transformd T_i_c(estimator.ric[0], estimator.tic[0]);
    Transformd T_w_l = T_w_i * T_i_c * Tcam_lidar;

    Quaterniond tmp_Q_prev;
    tmp_Q_prev = Quaterniond(estimator.Rs[WINDOW_SIZE - 2]);
    Transformd T_w_i_prev(tmp_Q_prev, estimator.Ps[WINDOW_SIZE - 2]);
    Transformd T_w_l_prev = T_w_i_prev * T_i_c * Tcam_lidar;

    nav_msgs::Odometry lidar_odometry;
    lidar_odometry.header = header;
    lidar_odometry.header.frame_id = LIO_World_Frame;
    lidar_odometry.child_frame_id = LIO_World_Frame;

    lidar_odometry.pose.pose.position.x = T_w_l.pos.x();
    lidar_odometry.pose.pose.position.y = T_w_l.pos.y();
    lidar_odometry.pose.pose.position.z = T_w_l.pos.z();
    lidar_odometry.pose.pose.orientation.x = T_w_l.rot.x();
    lidar_odometry.pose.pose.orientation.y = T_w_l.rot.y();
    lidar_odometry.pose.pose.orientation.z = T_w_l.rot.z();
    lidar_odometry.pose.pose.orientation.w = T_w_l.rot.w();
    lidar_odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
    lidar_odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
    lidar_odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();

    pub_loam_odometry.publish(lidar_odometry);

    geometry_msgs::PoseStamped visualLOAMPose;
    visualLOAMPose.header = lidar_odometry.header;
    visualLOAMPose.pose = lidar_odometry.pose.pose;
    violoamPath.header.stamp = lidar_odometry.header.stamp;
    violoamPath.header.frame_id = LIO_World_Frame;
    violoamPath.poses.push_back(visualLOAMPose);
    pub_vloamPath.publish(violoamPath);

    Transformd T_pre_cur = T_w_l_prev.inverse() * T_w_l;
    Vector3d delta_p = T_pre_cur.pos;
    Vector3d delta_v =
        estimator.Vs[WINDOW_SIZE] - estimator.Vs[WINDOW_SIZE - 2];
    ROS_DEBUG("\033[1;32m----> delta_p_v:%f,%f,%f|%f,%f,%f \033[0m", delta_p.x(), delta_p.y(), delta_p.z(), delta_v.x(), delta_v.y(), delta_v.z());
    ROS_DEBUG("norm   :%f|%f", delta_p.norm(), delta_v.norm());
    ROS_DEBUG("delta_t  :%f", dt);
    std_msgs::Float64 rosp;
    std_msgs::Float64 rosv;
    std_msgs::Float64 rost;
    rosp.data = delta_p.norm();
    rosv.data = delta_v.norm();
    rost.data = dt;
    pub_delta_p.publish(rosp);
    pub_delta_v.publish(rosv);
    pub_delta_t.publish(rost);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = VINS_World_Frame;
    pose_stamped.pose = odometry.pose.pose;
    path.header = header;
    path.header.frame_id = VINS_World_Frame;
    path.poses.push_back(pose_stamped);
    //"path"
    pub_path.publish(path);

    Vector3d correct_t;
    Vector3d correct_v;
    Quaterniond correct_q;
    correct_t = estimator.drift_correct_r * estimator.Ps[WINDOW_SIZE] +
                estimator.drift_correct_t;
    correct_q = estimator.drift_correct_r * estimator.Rs[WINDOW_SIZE];
    odometry.pose.pose.position.x = correct_t.x();
    odometry.pose.pose.position.y = correct_t.y();
    odometry.pose.pose.position.z = correct_t.z();
    odometry.pose.pose.orientation.x = correct_q.x();
    odometry.pose.pose.orientation.y = correct_q.y();
    odometry.pose.pose.orientation.z = correct_q.z();
    odometry.pose.pose.orientation.w = correct_q.w();

    pose_stamped.pose = odometry.pose.pose;
    relo_path.header = header;
    relo_path.header.frame_id = VINS_World_Frame;
    relo_path.poses.push_back(pose_stamped);
    //"relocalization_path"
    pub_relo_path.publish(relo_path);

    // write result to file
    ofstream foutC(VINS_RESULT_PATH, ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << header.stamp.toSec() * 1e9 << ",";
    foutC.precision(5);
    foutC << estimator.Ps[WINDOW_SIZE].x() << ","
          << estimator.Ps[WINDOW_SIZE].y() << ","
          << estimator.Ps[WINDOW_SIZE].z() << "," << tmp_Q.w() << ","
          << tmp_Q.x() << "," << tmp_Q.y() << "," << tmp_Q.z() << ","
          << estimator.Vs[WINDOW_SIZE].x() << ","
          << estimator.Vs[WINDOW_SIZE].y() << ","
          << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
    foutC.close();
  }
}

void pubKeyPoses(const MCVIOEstimator &estimator, const std_msgs::Header &header)
{
#if SHOW_LOG_DEBUG
  LOG(INFO) << "Pub key pose";
#endif
  if (estimator.key_poses.size() == 0)
    return;
  visualization_msgs::Marker key_poses;
  key_poses.header = header;
  key_poses.header.frame_id = VINS_World_Frame;
  key_poses.ns = Camera_Frame;
  key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
  key_poses.action = visualization_msgs::Marker::ADD;
  key_poses.pose.orientation.w = 1.0;
  key_poses.lifetime = ros::Duration();

  // static int key_poses_id = 0;
  key_poses.id = 0; // key_poses_id++;
  key_poses.scale.x = 0.05;
  key_poses.scale.y = 0.05;
  key_poses.scale.z = 0.05;
  key_poses.color.r = 1.0;
  key_poses.color.a = 1.0;

  for (int i = 0; i <= WINDOW_SIZE; i++)
  {
    geometry_msgs::Point pose_marker;
    Vector3d correct_pose;
    correct_pose = estimator.key_poses[i];
    pose_marker.x = correct_pose.x();
    pose_marker.y = correct_pose.y();
    pose_marker.z = correct_pose.z();
    key_poses.points.push_back(pose_marker);
  }
  //"key_poses"
  pub_key_poses.publish(key_poses);
}

void pubCameraPose(const MCVIOEstimator &estimator, const std_msgs::Header &header)
{
#if SHOW_LOG_DEBUG
  LOG(INFO) << "Pub camera Pose";
#endif
  int idx2 = WINDOW_SIZE - 1;

  if (estimator.solver_flag == MCVIOEstimator::SolverFlag::NON_LINEAR)
  {
    int i = idx2;
    Vector3d P = estimator.Ps[i];
    Matrix3d R = estimator.Rs[i];
    std_msgs::Header header_ = header;
    header_.frame_id = VINS_World_Frame;
    for (auto sensor : estimator.frontend_->sensors)
    {
      if (sensor->type == MCVIO::sensor_type::MONOCULAR)
      {
        std::shared_ptr<MCVIOcamera> cam = dynamic_pointer_cast<MCVIOcamera>(sensor);

        cam->pub_cam(P, R, header_);
      }
    }
  }
}

void pubSlideWindowPoses(const MCVIOEstimator &estimator,
                         const std_msgs::Header &header)
{
#if SHOW_LOG_DEBUG
  LOG(INFO) << "Pub sliding window pose";
#endif
  std_msgs::Header output_header = header;
  output_header.frame_id = VINS_World_Frame;
  // int local_window_size = 0;
  if (estimator.solver_flag == MCVIOEstimator::SolverFlag::NON_LINEAR)
  {

    for (auto sensor : estimator.frontend_->sensors)
    {
      if (sensor->type == MCVIO::sensor_type::MONOCULAR)
      {
        std::shared_ptr<MCVIOcamera> cam = dynamic_pointer_cast<MCVIOcamera>(sensor);

        // reset keyframebasevisual
        cam->keyframebasevisual.reset();
        int idx = estimator.frontend_->tracker_tag[cam->name];
        Transformd T_i_c(estimator.ric[idx], estimator.tic[idx]);
        for (const auto frame : estimator.localWindowFrames)
        {

          Transformd T_w_curi = frame.second.Twi[idx];
          Transformd T_w_curc = T_w_curi * T_i_c;

          Vector3d P = T_w_curc.pos;
          Quaterniond R = T_w_curc.rot;
          cam->keyframebasevisual.add_pose(P, R);
          cam->keyframebasevisual.publish_by(cam->pub_slidewindow_camera_pose, output_header);
          // local_window_size++;
        }
      }
    }
    //   LOG(INFO) << " estimator.localWindowFrames SIZE: " << local_window_size;
  }

} // pubSlideWindowPoses

void pubPointCloud(const MCVIOEstimator &estimator, const std_msgs::Header &header)
{
#if SHOW_LOG_DEBUG
          LOG(INFO)
      << "Pub point cloud";
#endif
  sensor_msgs::PointCloud point_cloud, loop_point_cloud;
  point_cloud.header = header;
  loop_point_cloud.header = header;
  for (int c = 0; c < NUM_OF_CAM; c++)
    for (auto &landmark : estimator.f_manager.KeyPointLandmarks[c])
    {
      int used_num = landmark.second.obs.size();
      auto host_tid = landmark.second.kf_id;

      auto host_id = estimator.time_frameid2_int_frameid.at(host_tid);

      if (!(used_num >= 2 && host_id < WINDOW_SIZE - 2))
        continue;
      if (host_id > WINDOW_SIZE * 3.0 / 4.0 ||
          landmark.second.solve_flag != KeyPointLandmark::SolveFlag::SOLVE_SUCC)
        continue;

      double depth = landmark.second.obs.at(host_tid).depth_measured;
      // camera coordinate
      Vector3d pts_i = depth == 0
                           ? landmark.second.obs.at(host_tid).point *
                                 landmark.second.estimated_depth
                           : landmark.second.obs.at(host_tid).point * depth;
      // C_imu = extrinsicRotation * C_cam + extrinsicTranslation
      // C_world = extrinsicRotation * C_imu + extrinsicTranslation
      Vector3d w_pts_i =
          estimator.Rs[host_id] * (estimator.ric[c] * pts_i + estimator.tic[c]) +
          estimator.Ps[host_id];
      // ROS_ERROR("x: %f   y: %f   z: %f   depth: %f",
      // pts_i(0),pts_i(1),pts_i(2),depth);
      geometry_msgs::Point32 p;
      p.x = w_pts_i(0);
      p.y = w_pts_i(1);
      p.z = w_pts_i(2);
      point_cloud.points.push_back(p);
    }
  //"point_cloud"
  pub_point_cloud.publish(point_cloud);

  // pub margined potin
  sensor_msgs::PointCloud margin_cloud;
  margin_cloud.header = header;
  for (int c = 0; c < NUM_OF_CAM; c++)
    for (auto &landmark : estimator.f_manager.KeyPointLandmarks[c])
    {

      int used_num = landmark.second.obs.size();
      auto host_tid = landmark.second.kf_id;
      auto host_id = estimator.time_frameid2_int_frameid.at(host_tid);

      if (!(used_num >= 2 && host_id < WINDOW_SIZE - 2))
        continue;

      // if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 ||
      // it_per_id->solve_flag != 1)
      //  continue;

      if (host_id == 0 && landmark.second.obs.size() <= 2 &&
          landmark.second.solve_flag == KeyPointLandmark::SolveFlag::SOLVE_SUCC)
      {

        double depth = landmark.second.obs.at(host_tid).depth_measured;
        Vector3d pts_i = depth == 0
                             ? landmark.second.obs.at(host_tid).point *
                                   landmark.second.estimated_depth
                             : landmark.second.obs.at(host_tid).point * depth;
        // std::cout<<"pts_i(0):"<<pts_i(0)<<"    pts_i(1):"<<pts_i(1)<<"
        // pts_i(2):"<<pts_i(2)<<std::endl;
        Vector3d w_pts_i = estimator.Rs[host_id] *
                               (estimator.ric[0] * pts_i + estimator.tic[0]) +
                           estimator.Ps[host_id];

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        margin_cloud.points.push_back(p);
      }
    }

  //"history_cloud"
  pub_margin_cloud.publish(margin_cloud);
}

// TODO: Not configured to multicam

// void pubWindowLidarPointCloud(const MCVIOEstimator &estimator,
//                               const std_msgs::Header &header)
// {
//   if (estimator.solver_flag == MCVIOEstimator::SolverFlag::NON_LINEAR)
//   {
//     pcl::PointCloud<pcl::PointXYZ> output;

//     pcl::PointCloud<pcl::PointXYZ> pc_in_new;
//     pcl::PointCloud<pcl::PointXYZ> tmp;
//     size_t num_points = 0;

//     for (const auto frame : estimator.localWindowFrames)
//     {
//       num_points += frame.second.cloud_ptr->size();
//     }
//     output.reserve(num_points);
//     pc_in_new.reserve(num_points);

//     Transformd T_w_newi(estimator.Rs[WINDOW_SIZE], estimator.Ps[WINDOW_SIZE]);
//     for (int c = 0; c < NUM_OF_CAM; c++)
//     {
//       Transformd T_i_c(estimator.ric[c], estimator.tic[c]);
//       Transformd T_w_newc = T_w_newi * T_i_c;
//       for (const auto frame : estimator.localWindowFrames)
//       {

//         tmp.resize(frame.second.cloud_ptr->size());
//         // Transformd T_new_i_cur_i = T_w_newi.inverse() * frame.second.Twi;
//         Transformd T_w_curi = frame.second.Twi[c];
//         Transformd T_w_curc = T_w_curi * T_i_c;
//         Transformd T_newc_curc = T_w_newc.inverse() * T_w_curc;
//         pcl::transformPointCloud(*frame.second.cloud_ptr, tmp,
//                                  T_w_curc.cast<float>().matrix());
//         output += tmp;

//         pcl::transformPointCloud(*frame.second.cloud_ptr, tmp,
//                                  T_newc_curc.cast<float>().matrix());
//         pc_in_new += tmp;
//       }
//     }
//     sensor_msgs::PointCloud2 rosmsg_output;
//     pcl::toROSMsg(output, rosmsg_output);
//     rosmsg_output.header = header;
//     pub_lidar_cloud.publish(rosmsg_output);

//     cv::Mat depth_image = estimator.localWindowFrames
//                               .at(estimator.Headers[WINDOW_SIZE].stamp.toSec())
//                               .image;

//     for (auto iter = pc_in_new.begin(); iter != pc_in_new.end(); ++iter)
//     {

//       //将点云的xyz,转换成uv
//       Eigen::Vector3d P{iter->x, iter->y, iter->z};
//       Eigen::Vector2d uv;
//       m_camera->spaceToPlane(P, uv);

//       //判断激光点云 是否在图像平面上
//       if (iter->z > 0 and MCVIOfrontend::is_in_image(uv, MCVIOfrontend::Boundary,
//                                                      1, nullptr))
//       { // && iter->z < 5.0

//         int u = static_cast<int>(uv(0));
//         int v = static_cast<int>(uv(1));

//         //设置最大最远距离
//         float z_min = 1.0;
//         float z_max = 50.0;
//         //设置距离差
//         float dz = z_max - z_min;
//         //取深度值
//         float z = iter->z;
//         //        if(v>30)
//         //        cout<<"v: "<<v<<endl;
//         float r = 1.0;
//         float g = 1.0;
//         float b = 1.0;
//         if (z < z_min)
//           z = z_min;
//         if (z > z_max)
//           z = z_max;

//         if (z < z_min + 0.25 * dz)
//         {
//           r = 0.0;
//           g = 4 * (z - z_min) / dz;
//         }
//         else if (z < (z_min + 0.5 * dz))
//         {
//           r = 0.0;
//           b = 1 + 4 * (z_min + 0.25 * dz - z) / dz;
//         }
//         else if (z < (z_min + 0.75 * dz))
//         {
//           r = 4 * (z - z_min - 0.5 * dz) / dz;
//           b = 0.0;
//         }
//         else
//         {
//           g = 1 + 4 * (z_min + 0.75 * dz - z) / dz;
//           b = 0.0;
//         }

//         cv::circle(depth_image, cv::Point(u, v), 3.5,
//                    cv::Scalar(static_cast<int>(r * 255),
//                               static_cast<int>(g * 255),
//                               static_cast<int>(b * 255)),
//                    -1);
//       }
//     }

//     sensor_msgs::ImagePtr depthImgMsg =
//         cv_bridge::CvImage(header, "bgr8", depth_image).toImageMsg();
//     pub_map_projection_image.publish(depthImgMsg);
//   }
// }

void pubTF(const MCVIOEstimator &estimator, const std_msgs::Header &header)
{
#if SHOW_LOG_DEBUG
          LOG(INFO)
      << "Pub TF";
#endif
  if (estimator.solver_flag != MCVIOEstimator::SolverFlag::NON_LINEAR)
    return;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  // body frame
  Vector3d correct_t;
  Quaterniond correct_q;
  correct_t = estimator.Ps[WINDOW_SIZE];
  correct_q = estimator.Rs[WINDOW_SIZE];

  transform.setOrigin(tf::Vector3(correct_t(0), correct_t(1), correct_t(2)));
  q.setW(correct_q.w());
  q.setX(correct_q.x());
  q.setY(correct_q.y());
  q.setZ(correct_q.z());
  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, header.stamp, VINS_World_Frame, VINS_IMU_Frame));

  // camera frame
  transform.setOrigin(tf::Vector3(estimator.tic[0].x(), estimator.tic[0].y(),
                                  estimator.tic[0].z()));
  q.setW(Quaterniond(estimator.ric[0]).w());
  q.setX(Quaterniond(estimator.ric[0]).x());
  q.setY(Quaterniond(estimator.ric[0]).y());
  q.setZ(Quaterniond(estimator.ric[0]).z());
  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, header.stamp, VINS_IMU_Frame, Camera_Frame));
  for (int c = 0; c < NUM_OF_CAM; c++)
  {
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = VINS_World_Frame;
    odometry.pose.pose.position.x = estimator.tic[c].x();
    odometry.pose.pose.position.y = estimator.tic[c].y();
    odometry.pose.pose.position.z = estimator.tic[c].z();
    Quaterniond tmp_q{estimator.ric[c]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic.publish(odometry);
  }
}

void pubKeyframe(const MCVIOEstimator &estimator)
{
#if SHOW_LOG_DEBUG
          LOG(INFO)
      << "Pub key frame";
#endif
  // pub camera pose, 2D-3D points of keyframe
  if (estimator.solver_flag == MCVIOEstimator::SolverFlag::NON_LINEAR &&
      estimator.marginalization_flag ==
          MCVIOEstimator::MarginalizationFlag::MARGIN_OLD)
  {
    int i = WINDOW_SIZE - 2;
    // Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
    Vector3d P = estimator.Ps[i];
    Quaterniond R = Quaterniond(estimator.Rs[i]);

    nav_msgs::Odometry odometry;
    odometry.header = estimator.Headers[WINDOW_SIZE - 2];
    odometry.header.frame_id = VINS_World_Frame;
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = R.x();
    odometry.pose.pose.orientation.y = R.y();
    odometry.pose.pose.orientation.z = R.z();
    odometry.pose.pose.orientation.w = R.w();
    // printf("time: %f t: %f %f %f r: %f %f %f %f\n",
    // odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(),
    // R.z()); "keyframe_pose"
    pub_keyframe_pose.publish(odometry);

    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = estimator.Headers[WINDOW_SIZE - 2];
    for (int c = 0; c < NUM_OF_CAM; c++)
      for (auto &landmark : estimator.f_manager.KeyPointLandmarks[c])
      {
        auto host_tid = landmark.second.kf_id;
        auto host_id = estimator.time_frameid2_int_frameid.at(host_tid);

        // 确保滑窗中two_newest 有观测值
        if (landmark.second.solve_flag ==
                KeyPointLandmark::SolveFlag::SOLVE_SUCC &&
            host_id < WINDOW_SIZE - 2 &&
            landmark.second.obs.find(estimator.int_frameid2_time_frameid.at(
                WINDOW_SIZE - 2)) != landmark.second.obs.end())
        {

          auto target_tid =
              estimator.int_frameid2_time_frameid.at(WINDOW_SIZE - 2);

          double depth = landmark.second.obs.at(host_tid).depth_measured;
          depth = depth == 0 ? landmark.second.estimated_depth : depth;

          {
            Vector3d pts_i = landmark.second.obs.at(host_tid).point * depth;
            Vector3d w_pts_i = estimator.Rs[host_id] *
                                   (estimator.ric[c] * pts_i + estimator.tic[c]) +
                               estimator.Ps[host_id];
            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            point_cloud.points.push_back(p);

            sensor_msgs::ChannelFloat32 p_2d;
            p_2d.values.push_back(landmark.second.obs.at(target_tid).point.x());
            p_2d.values.push_back(landmark.second.obs.at(target_tid).point.y());
            p_2d.values.push_back(landmark.second.obs.at(target_tid).uv.x());
            p_2d.values.push_back(landmark.second.obs.at(target_tid).uv.y());
            p_2d.values.push_back(landmark.second.feature_id);
            point_cloud.channels.push_back(p_2d);
          }
        }
      }

    //"keyframe_point"
    // now have more points than origin
    pub_keyframe_point.publish(point_cloud);
  }
}

void pubRelocalization(const MCVIOEstimator &estimator)
{
  nav_msgs::Odometry odometry;
  odometry.header.stamp = ros::Time(estimator.relo_frame_stamp);
  odometry.header.frame_id = VINS_World_Frame;
  odometry.pose.pose.position.x = estimator.relo_relative_t.x();
  odometry.pose.pose.position.y = estimator.relo_relative_t.y();
  odometry.pose.pose.position.z = estimator.relo_relative_t.z();
  odometry.pose.pose.orientation.x = estimator.relo_relative_q.x();
  odometry.pose.pose.orientation.y = estimator.relo_relative_q.y();
  odometry.pose.pose.orientation.z = estimator.relo_relative_q.z();
  odometry.pose.pose.orientation.w = estimator.relo_relative_q.w();
  odometry.twist.twist.linear.x = estimator.relo_relative_yaw;
  odometry.twist.twist.linear.y = estimator.relo_frame_index;
  //"relo_relative_pose"
  pub_relo_relative_pose.publish(odometry);
}