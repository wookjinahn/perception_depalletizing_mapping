# perception_depalletizing_mapping

This package is used to generate depalletizing map for depalletizing system on ROS einvironment.  
Package consists **depalletizing_mapping**.

Author: Wookjin Ahn  
Affiliation: CAMEL Lab.   
Maintainer : Wookjin Ahn, lexqrt2@pusan.ac.kr, Inho Lee, inholee8@pusan.ac.kr

# How to Install

## Dependencies
- [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)  (you shoud setup catkin_ws)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)  (using realsense d435)


## Install Package
The command below uses [catkin build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) of [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).  
you can use [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) instead of **catkin build**.
```bash
cd ~/catkin_ws/src
git clone https://github.com/PNUxCAMEL/camel-perception-depalletizingmap.git
catkin build heightmap --save-config --cmake-args -DCMAKE_BUILD_TYPE=Release
# using catkin_make instead of catkin build
# cd ~/catkin_ws/
# catkin_make -DCMAKE_BUILD_TYPE=Release
```

---
# How to Use

### 1. create catkin package
```bash
cd ~/catkin_ws/src/
catkin create pkg depalletizing_demo    # catkin_create_pkg using_msgs
cd using_msgs
clion .       # open CLion at package directory
```

### 2. create and config main.cpp, CMakeLists.txt, package.xml
Example Codes can be found at [here](https://github.com/PNUxCAMEL/camel-perception-heightmap/tree/main/heightmap_ros/example)  
Copy files _**depalletizing_demo.cpp, CMakeLists.txt, package.xml**_ into your catkin package directory.  

### 3. build and run
```bash
# before run, you should run realsense roslaunch.
roslaunch realsense2_camera demo_pointcloud.launch

cd ~/catkin_ws/src/using_msgs
catkin build depalletizing_demo        # catkin_make
rosrun depalletizing_demo depalletizing_demo
```

---