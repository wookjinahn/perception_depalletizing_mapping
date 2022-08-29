# perception_depalletizing_mapping

![depalletizing_mapping](depalletizing_mapping_demos/data/depalletizing_mapping.gif)

This package is used to generate depalletizing map for depalletizing system on ROS environment.  
Package consists **depalletizing_mapping**.

Author: Wookjin Ahn  
Affiliation: CAMEL Lab @ Pusan National University.   
Maintainer : Wookjin Ahn, lexqrt2@pusan.ac.kr, Inho Lee, inholee8@pusan.ac.kr

---
# How to Install

## Dependencies
You MUST install dependencies below first.
- [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)  (you should setup catkin_ws)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)  (using realsense d435)


## Install Package 
you use [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) for build package.
```bash
cd ~/catkin_ws/src/
git clone https://github.com/wookjinAhn/perception_depalletizing_mapping.git
catkin_make -DCMAKE_BUILD_TYPE=Release
```

---
# How to run demo
You can find the demo usage [here](https://github.com/wookjinAhn/perception_depalletizing_mapping/tree/master/depalletizing_mapping_demos)

---
# Primary Functions of each class.
### MapDataNodeROS : contains data to be used for the process and is used for input and output.
**MapDataNodeROS() :** is a basic constructor. It can construct with basic ransac object which threshold is 0.005m (5mm).   
**MapDataNodeROS(Ransac& ransac) :** is a constructor with specified ransac object.  
**void SetCameraHeight(float cameraHeight) :** is an essential function to use this package. You have to check you camera position height.  
**void SetRansac(Ransac& ransac) :** is a function to set a specific ransac after constructed MapDataNodeROS object.  
**void FromPointCloud2Msgs(sensor_msgs::PointCloud2 pointcloud2_msgs) :** is used to read pointcloud2 data from d435 before processing.   
**std::vector<float> ToPointCloud2Msgs(std::string frame_id, sensor_msgs::PointCloud2& output_pointcloud) :** is used to publish the detected planner region as a pointcloud2 message after processing.     
**void ToDepalletizingMapMsgs(std::string frame_id, depalletizing_mapping_msgs::DepalletizingMap& depalletizing_mapping_msgs) :** is used to publish the depalletizing_mapping message after processing.

### MapTreeNodeROS : is using for data processing.
**MapTreeNodeROS(MapDataNodeROS*** **mapDataNode) :** is a constructor with the MapDataNodeROS object as declared earlier.   
**MapTreeNodeROS(MapDataNodeROS*** **mapDataNode, BoundingBox boundingBox) :** is a constructor with the MapDataNodeROS object as declared earlier and BoundingBox object.

### BoundingBox : set the data range to be used as the class used in MapTreeNodeROS.
**BoundingBox() :** is a basic constructor with range (0, 0, 1, 1). It is set based on the camera coordinate system.   
**BoundingBox(float x, float z, float w, float h) :** is constructor with specified range value. x and y is center point of data range. w is width and h is height value, which determine how far away from the center point xy to range.   
**void SetBoundary(float x, float z, float w, float h) :** can set specified range value.    

### Ransac : is using for planar region detection
**Ransac() :** is a basic constructor. It can construct with basic planar model object and has 0.005m (5mm) threshold. max iteration value is decided with input points size.     
**Ransac(PlaneModel& model, float modelThreshold, int maxIteration) :** is constructor with specified model for detecting, threshold and iteration values.  
**Ransac(PlaneModel& model, float modelThreshold) :** is constructor with specified model for detecting and threshold value.  
**Ransac(float modelThreshold) :** is constructor with specified threshold value.
**void SetModelThreshold(float modelThreshold) :** is used to set threshold value.  
**void SetMaxIteration(int maxiteration) :** is used to set max iteration value.   
**std::vector<PlaneModel> GetResultModel() const :** is used to get model data after RANSAC processing that is befitting.    
**std::vector<Point3D> GetResultData() const :** is used to get data points of result model.  