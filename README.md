# perception_depalletizing_mapping

![depalletizing_mapping](depalletizing_mapping_demos/data/depalletizing_mapping.gif)

This package is used to generate depalletizing map for depalletizing system on ROS einvironment.  
Package consists **depalletizing_mapping**.

Author: Wookjin Ahn  
Affiliation: CAMEL Lab @ Pusan National University.   
Maintainer : Wookjin Ahn, lexqrt2@pusan.ac.kr, Inho Lee, inholee8@pusan.ac.kr

# How to Install

## Dependencies
You MUST install dependencies below first.
- [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)  (you shoud setup catkin_ws)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)  (using realsense d435)


## Install Package
The command below uses [catkin build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) of [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).  
you can use [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) instead of **catkin build**.
```bash
cd ~/catkin_ws/src
git clone https://github.com/wookjinAhn/perception_depalletizing_mapping.git
catkin build depalletizing_mapping --save-config --cmake-args -DCMAKE_BUILD_TYPE=Release
# using catkin_make instead of catkin build
# cd ~/catkin_ws/
# catkin_make -DCMAKE_BUILD_TYPE=Release
```

---
# How to Use
### 1. Copy .launch file and .rviz file into realsense-ros package
Files location :  
depalletizing_mapping_demos/launch/**depalletizing_demo.launch**  
depalletizing_mapping_demos/rviz/**depalletizing_demo.rviz**
```bash
# copy depalletizing_demo.launch
cd ~/catkin_ws/src/perception_depalletizing_mapping/depalletizing_mapping_demos/launch/
cp depalletizing_demo.launch ~/catkin_ws/src/realsense-ros/realsense2_camera/launch/

# copy depalletizing_demo.launch
cd ~/catkin_ws/src/perception_depalletizing_mapping/depalletizing_mapping_demos/rviz/
cp depalletizing_demo.rviz ~/catkin_ws/src/realsense-ros/realsense2_camera/rviz/
```

### 2. Create catkin package
```bash
cd ~/catkin_ws/src/
catkin create pkg depalletizing_demo    # catkin_create_pkg using_msgs
cd depalletizing_demo
clion .                                 # open CLion at package directory
```

### 3. Create and config main.cpp, CMakeLists.txt, package.xml
Example Codes can be found at [here](https://github.com/wookjinAhn/perception_depalletizing_mapping/tree/master/depalletizing_mapping_demos)  
Copy files into your catkin package directory.  
**include/DepalletizingMappingDemo.hpp, src/DepalletizingMappingDemo.cpp, src/depalletizing_demo.cpp, CMakeLists/CMakeLists.txt, pacakgexml/package.xml**   
```bash
cd ~/catkin_ws/src/depalletizing_demo
mkdir include
mkdir src

# copy DepalletizingMappingDemo.hpp
cd ~/catkin_ws/src/perception_depalletizing_mapping/depalletizing_mapping_demos/include/
cp DepalletizingMappingDemo.hpp ~/catkin_ws/src/depalletizing_demo/include/

# copy DepalletizingMappingDemo.cpp
cd ~/catkin_ws/src/perception_depalletizing_mapping/depalletizing_mapping_demos/src/
cp DepalletizingMappingDemo.cpp ~/catkin_ws/src/depalletizing_demo/src/

# copy depalletizing_demo.cpp
cd ~/catkin_ws/src/perception_depalletizing_mapping/depalletizing_mapping_demos/src/
cp depalletizing_demo.cpp ~/catkin_ws/src/depalletizing_demo/src/

# copy CMakeLists.txt
cd ~/catkin_ws/src/perception_depalletizing_mapping/depalletizing_mapping_demos/CMakeLists/
cp CMakeLists.txt ~/catkin_ws/src/depalletizing_demo/

# copy package.xml
cd ~/catkin_ws/src/perception_depalletizing_mapping/depalletizing_mapping_demos/packagexml/
cp package.xml ~/catkin_ws/src/depalletizing_demo/
```

### 4. Build and run
```bash
# before run, you should run realsense roslaunch.
roslaunch realsense2_camera depalletizing_demo.launch

cd ~/catkin_ws/src/depalletizing_demo
catkin build depalletizing_demo        # catkin_make
rosrun depalletizing_demo depalletizing_demo
```

---
