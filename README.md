# perception_depalletizing_mapping

![depalletizing_mapping](depalletizing_mapping_demos/data/depalletizing_mapping.gif)

This package is used to generate depalletizing map for depalletizing system on ROS einvironment.  
Package consists **depalletizing_mapping**.

Author: Wookjin Ahn  
Affiliation: CAMEL Lab @ Pusan National University.   
Maintainer : Wookjin Ahn, lexqrt2@pusan.ac.kr, Inho Lee, inholee8@pusan.ac.kr

---
# How to Install

## Dependencies
You MUST install dependencies below first.
- [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)  (you shoud setup catkin_ws)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)  (using realsense d435)


## Install Package 
you use [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) for build package.
```bash
cd ~/catkin_ws/src
git clone https://github.com/wookjinAhn/perception_depalletizing_mapping.git
catkin_make -DCMAKE_BUILD_TYPE=Release
```

---
# How to run demo
You can find the demo usage [here](https://github.com/wookjinAhn/perception_depalletizing_mapping/tree/master/depalletizing_mapping_demos)

---
# Primary Functions
### MapDataNodeROS
### MapTreeNodeROS