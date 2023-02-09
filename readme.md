# MCVIO: Multi-Camera Visual-Inertial Odometry

## Toward Robust Visual-Inertial Odometry with Multiple Nonoverlapping Monocular Cameras

We present a Visual-Inertial Odometry (VIO)
algorithm with multiple non-overlapping monocular cameras
aiming at improving the robustness of the VIO algorithm.
An initialization scheme and tightly-coupled bundle adjustment
for multiple non-overlapping monocular cameras are proposed.
With more stable features captured by multiple cameras, VIO
can maintain stable state estimation, especially when one of
the cameras tracked unstable or limited features. We also
address the high CPU usage rate brought by multiple cameras
by proposing a GPU-accelerated frontend. Finally, we use
our pedestrian carried system to evaluate the robustness of
the VIO algorithm in several challenging environments. The
results show that the multi-camera setup yields significantly
higher estimation robustness than a monocular system while
not increasing the CPU usage rate.

**Related Papers**

* **Toward Robust Visual-Inertial Odometry with Multiple Nonoverlapping Monocular Cameras**, Yao He, Huai Yu, Wen Yang, and Sebastian Scherer, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS, 2022), [pdf](https://github.com/shockwaveHe/MCVIO/blob/main/MCVIO/config/0707.pdf)

* Our work is based on: **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen, IEEE Transactions on Robotics [pdf](https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert) 


**Videos:**

<a href="https://www.youtube.com/watch?v=r7QvPth1m10" target="_blank"><img src="https://www.youtube.com/watch?v=r7QvPth1m10/0.jpg" 
alt="MCVIO" width="240" height="180" border="10" /></a>

## 1. Prerequisites
1.1 **Ubuntu** and **ROS**

Ubuntu  20.04.
ROS Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
additional ROS pacakge
```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```
To run the program on older Ubuntu version, the headers for OpenCV in the source code should be adjusted to the correct versions. 

1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.
(Our testing environment: Ubuntu 20.04, ROS Noetic, OpenCV 4.2, Eigen 3.3.3, Ceres 2.0.0, C++17) 

1.3. **Vision Programming Interface (VPI)**

Follow [VPI installation](https://docs.nvidia.com/vpi/installation.html).
(Our testing environment: VPI 1.0, it should work for VPI 2.0. If your are using version above 1.0, the configuration for VPI in CMakeLists should be adjusted. There are some bugs when using the VPI Pyramidal LK Optical Flow. Our codes already consider these bugs. If VPI resolve the bugs, remember to fix the code in vpi_feature_tracker.)

## 2. Build MCVIO on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/shockwaveHe/MCVIO.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Run MCVIO
```
    roslaunch mcvio MCVIO_estimator.launch 
    cd src/MCVIO/launch/
    rviz -d MCVIO_VIEW.launch
    rosbag play <Your path to dataset>/<Your bag file>.bag 
```

## 4. Datasets
4.1 Public datasets

Our MCVIO was tested on the [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).

4.2 Our own collected datasets

Due to storage issue, we currently can only provide one bag file [indoor_1.bag](https://drive.google.com/file/d/1tgq2AQ8pjkjaF8iNTKtnjAbXENSp33v3/view?usp=share_link). More datasets will be released later.

## 5. Citations

If you use MCVIO for your academic research, please cite our paper.
```
@inproceedings{Yao2022IROS,
  title={Toward Robust Visual-Inertial Odometry with Multiple Nonoverlapping Monocular Cameras},
  author={He, Yao and Yu, Huai and Yang, Wen and Scherer, Sebastian},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2022},
  organization={IEEE}
}
```

## 6. Notes for Usage and Future Development

6.1 We use the ***frame_id*** in ROS messages as identifier for different sensors. Please provide the correct ***frame_id*** to ***sensor_list*** in the configuration. If there are different sensors having the same id or missing the id, errors may occur.

6.2 Our work highlight the scalability in terms of adding more sensors. We define the ***MCVIOsensor*** and ***TrackerBase*** class for future developers to easily add new type of sensors and behaviors through inherentance. We already use this feature to develop a multi-spectral inertial odometry. The latest work will be released soon! 
