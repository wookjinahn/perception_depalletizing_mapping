//
// Created by wj on 22. 5. 16.
//

// ===== Notice ====
// You should run "test_multi_without_tf" which placed in sample directory
// copy and paste at catkin_ws/src/realsense-ros/realsense2_camera/launch/
// roslaunch realsense2_camera test_multi_without_tf

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <heightmap_core/heightmap_core.hpp>
#include "heightmap_ros/heightmap_ros.hpp"

ros::Publisher pub1; //
ros::Publisher pub2; //
ros::Publisher pub3; //
ros::Publisher pub4; //

void pc_callback1 (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
{
	camel::MapDataNodeROS dataNode;
	camel::MapTreeNodeROS treeNode(&dataNode);

	// convert for use XYZ
	sensor_msgs::PointCloud pointcloud_msg;
	sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud2_msg, pointcloud_msg);

	// get points
	dataNode.FromMessage(pointcloud_msg);

	// sampling and rotation
	float rotationDegree = -45.0f;
	std::vector<camel::Point3*> samplingPoints = dataNode.SamplingPointsWithRotate(5000, rotationDegree);

	// insert points to tree and create 2.5d data
	treeNode.InsertMapTreeNode(samplingPoints);

	// declare for publish
	sensor_msgs::PointCloud output_pointcloud;
	sensor_msgs::PointCloud2 output_pointcloud2;

	// convert 2.5d data to sensor_msgs::Pointcloud2
	std::string fram_id = "camera1_depth_optical_frame";
	dataNode.ToMessage(fram_id, output_pointcloud);
	sensor_msgs::convertPointCloudToPointCloud2(output_pointcloud, output_pointcloud2);

	// pub
	pub1.publish(output_pointcloud2);

	std::cout << "camera1 : " << ros::Time::now() << std::endl;     // for check publish well
}

void pc_callback2 (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
{
	camel::MapDataNodeROS dataNode;
	camel::MapTreeNodeROS treeNode(&dataNode);

	// convert for use XYZ
	sensor_msgs::PointCloud pointcloud_msg;
	sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud2_msg, pointcloud_msg);

	// get points
	dataNode.FromMessage(pointcloud_msg);

	// sampling and rotation
	float rotationDegree = -45.0f;
	std::vector<camel::Point3*> samplingPoints = dataNode.SamplingPointsWithRotate(5000, rotationDegree);

	// insert points to tree and create 2.5d data
	treeNode.InsertMapTreeNode(samplingPoints);

	// declare for publish
	sensor_msgs::PointCloud output_pointcloud;
	sensor_msgs::PointCloud2 output_pointcloud2;

	// convert 2.5d data to sensor_msgs::Pointcloud2
	std::string fram_id = "camera2_depth_optical_frame";
	dataNode.ToMessage(fram_id, output_pointcloud);
	sensor_msgs::convertPointCloudToPointCloud2(output_pointcloud, output_pointcloud2);

	// pub
	pub2.publish(output_pointcloud2);

	std::cout << "camera22 : " << ros::Time::now() << std::endl;     // for check publish well
}

void pc_callback3 (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
{
	camel::MapDataNodeROS dataNode;
	camel::MapTreeNodeROS treeNode(&dataNode);

	// convert for use XYZ
	sensor_msgs::PointCloud pointcloud_msg;
	sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud2_msg, pointcloud_msg);

	// get points
	dataNode.FromMessage(pointcloud_msg);

	// sampling and rotation
	float rotationDegree = -45.0f;
	std::vector<camel::Point3*> samplingPoints = dataNode.SamplingPointsWithRotate(5000, rotationDegree);

	// insert points to tree and create 2.5d data
	treeNode.InsertMapTreeNode(samplingPoints);

	// declare for publish
	sensor_msgs::PointCloud output_pointcloud;
	sensor_msgs::PointCloud2 output_pointcloud2;

	// convert 2.5d data to sensor_msgs::Pointcloud2
	std::string fram_id = "camera3_depth_optical_frame";
	dataNode.ToMessage(fram_id, output_pointcloud);
	sensor_msgs::convertPointCloudToPointCloud2(output_pointcloud, output_pointcloud2);

	// pub
	pub3.publish(output_pointcloud2);

	std::cout << "camera333 : " << ros::Time::now() << std::endl;     // for check publish well
}

void pc_callback4 (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
{
	camel::MapDataNodeROS dataNode;
	camel::MapTreeNodeROS treeNode(&dataNode);

	// convert for use XYZ
	sensor_msgs::PointCloud pointcloud_msg;
	sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud2_msg, pointcloud_msg);

	// get points
	dataNode.FromMessage(pointcloud_msg);

	// sampling and rotation
	float rotationDegree = -45.0f;
	std::vector<camel::Point3*> samplingPoints = dataNode.SamplingPointsWithRotate(5000, rotationDegree);

	// insert points to tree and create 2.5d data
	treeNode.InsertMapTreeNode(samplingPoints);

	// declare for publish
	sensor_msgs::PointCloud output_pointcloud;
	sensor_msgs::PointCloud2 output_pointcloud2;

	// convert 2.5d data to sensor_msgs::Pointcloud2
	std::string fram_id = "camera4_depth_optical_frame";
	dataNode.ToMessage(fram_id, output_pointcloud);
	sensor_msgs::convertPointCloudToPointCloud2(output_pointcloud, output_pointcloud2);

	// pub
	pub4.publish(output_pointcloud2);

	std::cout << "camera4444 : " << ros::Time::now() << std::endl;     // for check publish well
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pub_test");

	auto nh = ros::NodeHandle();
	pub1 = nh.advertise<sensor_msgs::PointCloud2>("heightmap2topic1", 10);
	pub2 = nh.advertise<sensor_msgs::PointCloud2>("heightmap2topic2", 10);
	pub3 = nh.advertise<sensor_msgs::PointCloud2>("heightmap2topic3", 10);
	pub4 = nh.advertise<sensor_msgs::PointCloud2>("heightmap2topic4", 10);

	ros::Subscriber sub1 = nh.subscribe("/camera1/depth/color/points", 1, pc_callback1);
	ros::Subscriber sub2 = nh.subscribe("/camera2/depth/color/points", 1, pc_callback2);
	ros::Subscriber sub3 = nh.subscribe("/camera3/depth/color/points", 1, pc_callback3);
	ros::Subscriber sub4 = nh.subscribe("/camera4/depth/color/points", 1, pc_callback4);

	ros::spin();
}