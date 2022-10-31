#include "Frontend/sensors.h"
#include "Frontend/MCVIOfrontend.h"
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
using namespace std;
using namespace MCVIO;


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        LOG(INFO) << "Loaded " << name << ": " << ans;
    }
    else
    {
        LOG(ERROR) << "Failed to load " << name;
        n.shutdown();
    }
    return ans;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    string config_file = readParam<string>(n,"config_file");
    LOG(INFO)<<"Config_file:"<<config_file;
    
    MCVIOfrontend frontend(config_file);
    MCVIOfrontend_ = &frontend;
    frontend.setUpROS(nullptr, &n);
    ros::spin();

    return 1;
}