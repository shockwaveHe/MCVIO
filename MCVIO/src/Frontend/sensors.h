#ifndef SENSORS_H
#define SENSORS_H
#include <mutex>
#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <string>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "../utility/CameraPoseVisualization.h"
#include "MCVIOfrontend_data.h"
#include <map>
#include <Eigen/Dense>

#define SHOW_LOG_DEBUG 0

using namespace std;
typedef pcl::PointXYZ PointType;
namespace MCVIO
{
    enum sensor_type
    {
        MONOCULAR,
        UNKNOWN
    };

    struct CameraProcessingResults
    {
        double timestamp;
        FeatureTrackerResults features;
    };
    struct SyncCameraProcessingResults
    {
        double sync_timestamp;
        std::vector<std::shared_ptr<CameraProcessingResults>> results;
        std::vector<bool> isEmpty;
    };
    class FrontEndResultsSynchronizer
    {
    public:
        FrontEndResultsSynchronizer();
        ~FrontEndResultsSynchronizer(){};
        bool Sync();
        void addPool(std::string cam_name);
        void resize(size_t size);
        double timestamp;

        typedef std::shared_ptr<FrontEndResultsSynchronizer> Ptr;
        std::vector<std::shared_ptr<std::mutex>> result_mutexes;
        std::vector<std::shared_ptr<std::queue<std::shared_ptr<CameraProcessingResults>>>> results;
        // std::vector<std::pair<string, std::shared_ptr<CameraProcessingResults>>> current_result;
        unordered_map<string, int> tracker_tag;
        std::queue<std::shared_ptr<SyncCameraProcessingResults>> sync_results;
    };

    class MCVIOsensor
    {
    public:
        MCVIOsensor(sensor_type stype,
                    string topic,
                    string name,
                    ros::NodeHandle *node,
                    Eigen::Matrix3d R,
                    Eigen::Vector3d T);
        virtual ~MCVIOsensor(){};

        // virtual void* ptr() = 0;

        sensor_type type;
        string topic, name;
        ros::Subscriber sub;
        ros::NodeHandle *frontend_node;
        Eigen::Matrix3d ext_R; // extrinsic rotation
        Eigen::Vector3d ext_T; // extrinsic translation
    };

    // Monocular
    class MCVIOcamera : public MCVIOsensor
    {
    public:
        MCVIOcamera(sensor_type type,
                    string topic,
                    string name,
                    ros::NodeHandle *node,
                    Eigen::Matrix3d R,
                    Eigen::Vector3d T,
                    double fx, double fy, double cx, double cy, bool fisheye,
                    int w, int h);
        virtual ~MCVIOcamera(){};
        // MCVIOcamera* ptr(){return this;};
        bool setFisheye(string fisheye_path);
        void init_visualization();
        // P: imu P
        // R: imu R
        void pub_cam(Eigen::Vector3d & P, Eigen::Matrix3d & R, std_msgs::Header &header);

        bool FISHEYE;
        double fx, fy, cx, cy;
        int ROW, COL;
        cv::Mat mask;

        int MAX_CNT = 200;
        int MIN_DIST = 20;
        int FREQ = 10;

        double F_THRESHOLD = 1.0;
        bool EQUALIZE = 1;
        bool USE_VPI;
        int tracker_idx;

        bool first_image_flag = false;
        double first_image_time, last_image_time;
        int pub_count = 0;
        bool PUB_THIS_FRAME = false;
        int init_pub = 0;


        ros::Publisher pub_match;

        // For visualization 
        bool visualize = true;
        ros::Publisher pub_cam_pose, pub_cam_pose_visual, pub_slidewindow_camera_pose;
        CameraPoseVisualization cameraposevisual, keyframebasevisual;
    };

} // namespace MCVIO
#endif // SENSORS_H