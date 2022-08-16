# camel-perception-heightmap

<img src="https://user-images.githubusercontent.com/79748805/174438497-d5cd0893-ef05-4744-bf71-15f54b4b3f44.png" width="783" height="540"/>


This package is used to generate Heightmap with 2.5D map information for quadruped robots on ROS einvironment.  
Package contains **heightmap_core**, **heightmap_ros** with sample codes.

Author: Wookjin Ahn  
Affiliation: CAMEL Lab.   
Maintainer : Wookjin Ahn, lexqrt2@pusan.ac.kr, Inho Lee, inholee8@pusan.ac.kr

# How to Install

## Dependencies
- [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)  (you shoud setup catkin_ws)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)  (using realsense d435)
- [camel-euclid](https://github.com/PNUxCAMEL/camel-euclid)  (using Point2D and Point3D)  


## Install Package
The command below uses [catkin build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) of [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).  
you can use [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) instead of **catkin build**.
```bash
cd ~/catkin_ws/src
git clone https://github.com/PNUxCAMEL/camel-perception-heightmap.git
catkin build depalletizing_mapping --save-config --cmake-args -DCMAKE_BUILD_TYPE=Release
# using catkin_make instead of catkin build
# cd ~/catkin_ws/
# catkin_make -DCMAKE_BUILD_TYPE=Release
```
![Screenshot from 2022-05-20 14-20-16](https://user-images.githubusercontent.com/79748805/169456018-b8f3a6c8-8c67-408b-aa08-836b05467e82.png)

---
# How to Use

## heightmap_core
This package consists of classes that are the core of heightmap.  
You can use this package to create heightmap by reading a .PCD file contained pointcloud data.  

### 1. create catkin package
```bash
cd ~/catkin_ws/src/
catkin create pkg using_pcd    # catkin_create_pkg using_pcd
cd using_pcd
clion .       # open CLion at package directory
```

### 2. create and config main.cpp, CMakeLists.txt, package.xml  
Example can be found [here](https://github.com/PNUxCAMEL/camel-perception-heightmap/tree/main/heightmap_core/example)   
Copy files _**using_pcd.cpp, CMakeLists.txt, package.xml**_ into your catkin package directory.

### 3. build and run
```bash
cd ~/catkin_ws/src/using_pcd
catkin build using_pcd        # catkin_make
rosrun using_pcd using_pcd
```



## heightmap_ros
This package consists of the class of heightmap used in ROS.  
The basic usage is the same as the above heightmap_core package.  

### 1. create catkin package
```bash
cd ~/catkin_ws/src/
catkin create pkg using_msgs    # catkin_create_pkg using_msgs
cd using_msgs
clion .       # open CLion at package directory
```

### 2. create and config main.cpp, CMakeLists.txt, package.xml
Example Codes can be found at [here](https://github.com/PNUxCAMEL/camel-perception-heightmap/tree/main/heightmap_ros/example)  
Copy files _**using_msgs.cpp, CMakeLists.txt, package.xml**_ into your catkin package directory.  

### 3. build and run
```bash
# before run, you should run realsense roslaunch.
roslaunch realsense2_camera demo_pointcloud.launch

cd ~/catkin_ws/src/using_msgs
catkin build using_msgs        # catkin_make
rosrun using_msgs using_msgs
```

---
# Package Class Configuration
  
- **heightmap_core**
  + Point2D
  + Point3
  + NodeBoundary
  + MapTreeNode
  + MapDataNode


- **heightmap_ros**
  + MapTreeNodeROS
  + MapDataNodeROS

---
# Primary Variable and Parameters
### MapDataNode(ROS)
- _**InputPoints**_ contains pointcloud data get from PCD file or sensor_msgs::PointCloud2
- _**OutputPoints**_ contains pointcloud data for write PCD file or publish sensor_msgs::PointCloud2
- _**RotationDegree**_ is used at function _SamplingPointsWithRotate()_. This parameter should be set in degrees, not radians.
  As the degree value, the angle at which the camera is attached to the robot is usually used.
- _**SamplingNumber**_  is used at function _SamplingPointse()_ and _SamplingPointsWithRotate()_.
The data processing speed can be improved by sampling the input data using this parameter. Ideally, it should be set between 10,000 and 5,000.

### MapTreeNode(ROS)
- _**MapDataNode**_ contains MapDataNode class with the same name. 
- _**NodeBoundary**_ is using for quadtree configuration. Default value is width : 2m, height : 2m. 
You can set your own Boundary range using function _SetBoundary()_ or at constructor.
- _**Depth**_ is using for quadtree configuration. Default value is 6. When Boundary is 2 m X 2 m with Depth is 6, resolution becomes 3.125 cm.
You can set your own Depth value using function _SetDepth()_ or at constructor.
- _**Capacity**_ is using for quadtree configuration. Default value is 1. When points are inserted into a tree node, the node is subdivided if they exceed the corresponding Capacity value.
You can set your own Capacity value sing function _SetCapacity()_ or at constructor.


---
# Primary Function
### MapDataNode(ROS)
- Constructor
  + _**MapDataNode()**_
- _**FromPCD(**_ _String filePath_ _**)**_ 
- _**ToPCD(**_ _String filePath_ _**)**_ 
- _**FromMessage(**_ _sensor_msgs::PointCloud pcMsgs_ _**)**_
- _**ToMessage(**_ _sensor_msgs::PointCloud& pcMsgs_ _**)**_
- _**SamplingPointsWithRotate(**_ _int SamplingNum, float rotationDegree_ _**)**_
- _**SamplingPoints(**_ _int SamplingNum_ _**)**_




### MapTreeNode(ROS)
- Constructor
  + **MapTreeNode()**
  + _**MapTreeNode(**_ _MapDataNode* mapDataNode_ _**)**_
  + _**MapTreeNode(**_ _NodeBoundary boundary, int depth, int capacity_ _**)**_
  + _**MapTreeNode(**_ _NodeBoundary boundary, int depth_ _**)**_
  + _**MapTreeNode(**_ _**NodeBoundary boundary**_ _**)**_
- _**SetMapDataNode(**_ _MapDataNode* heightmap_ _**)**_ & _**GetMapDataNode()**_
- _**SetBoundary(**_ _NodeBoundary boundary_ _**)**_  & _**GetBoundary()**_
- _**SetBoundaryKey(**_ _float x, float z, float w, float h_ _**)**_
- _**SetDepth(**_ _int depth_ _**)**_
- _**SetCapacity(**_ _int capacity_ _**)**_
- _**InsertMapTreeNode(**_ _std::vector<Point3*> points_ _**)**_
