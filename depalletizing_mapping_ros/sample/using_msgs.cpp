//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/point_cloud_conversion.h>
//#include <depalletizing_mapping_core/depalletizing_mapping_core.hpp>
//#include <depalletizing_mapping_ros/depalletizing_mapping_ros.hpp>
//#include <tf/transform_broadcaster.h>
//
//ros::Publisher heightmap_pub; //
//ros::Publisher pointcloud_pub; //
//
//void cb_heightmap (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
//{
//	ros::Rate rate(1);
//	static tf::TransformBroadcaster tfBroadcaster;
//	tf::Transform transform;
//	transform.setOrigin(tf::Vector3(0, 0, 0));
//	tf::Quaternion quaternion;
//	quaternion.setRPY(0, 0, 0);
//	transform.setRotation(quaternion);
//
//	// create DataNode and TreeNode
//	camel::MapDataNodeROS dataNode;
//	camel::MapTreeNodeROS treeNode(&dataNode);
//
//	// convert for use XYZ
//	sensor_msgs::PointCloud pointcloud_msg;
//	sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud2_msg, pointcloud_msg);
//
//	// get points
//	dataNode.FromMessage(pointcloud_msg);
//
//	// sampling and rotation
////	std::vector<camel::Point3*> samplingPoints = dataNode.SamplingPointsWithRotate(5000, rotationDegree);
//	std::vector<camel::Point3*> samplingPoints = dataNode.SamplingPoints(5000);
//
//	// insert points to tree and create 2.5d data
//	treeNode.InsertMapTreeNode(samplingPoints);
//
//	// declare for publish
//	depalletizing_mapping_msgs::Heightmap output_heightmap;
//	dataNode.ToHeightmapMsgs(output_heightmap);
//
//	// pub
//	heightmap_pub.publish(output_heightmap);
//	tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "d435_depth_optical_frame", "/heightmap_frame"));
//
//	rate.sleep();
//	std::cout << ros::Time::now() << std::endl;     // for check publish well
//}
//
//void cb_pointcloud (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
//{
//	std::cout << "ww" << std::endl;
//
//	ros::Rate rate(1);
//	// create DataNode and TreeNode
//	camel::MapDataNodeROS dataNode;
//	camel::MapTreeNodeROS treeNode(&dataNode);
//
//	// convert for use XYZ
//	sensor_msgs::PointCloud pointcloud_msg;
//	sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud2_msg, pointcloud_msg);
//
//	// get points
//	dataNode.FromMessage(pointcloud_msg);
//	std::cout << dataNode.GetInputPoints().size() << std::endl;
//
//	// sampling and rotation
//	std::vector<camel::Point3*> samplingPoints = dataNode.SamplingPoints(5000);
//
//	// insert points to tree and create 2.5d data
//	treeNode.InsertMapTreeNode(samplingPoints);
//
//	sensor_msgs::PointCloud output_pointcloud;
//	sensor_msgs::PointCloud2 output_pointcloud2;
//	std::string fram_id = "d435_depth_optical_frame";
//	dataNode.ToMessage(fram_id, output_pointcloud);
//
//	sensor_msgs::convertPointCloudToPointCloud2(output_pointcloud, output_pointcloud2);
//	pointcloud_pub.publish(output_pointcloud2);
//
//
//	rate.sleep();
//	std::cout << ros::Time::now() << std::endl;     // for check publish well
//}
//
//int main(int argc, char** argv)
//{
//	ros::init(argc, argv, "pub_test");
//
//	auto heightmap_nh = ros::NodeHandle();
////	heightmap_pub = heightmap_nh.advertise<depalletizing_mapping_msgs::Heightmap>("depalletizing_mapping_msgs", 10);
////	ros::Subscriber heightmap_sub = heightmap_nh.subscribe("/d435/depth/color/points", 1, cb_heightmap);
//
//	auto pointcloud_nh = ros::NodeHandle();
//	pointcloud_pub = pointcloud_nh.advertise<sensor_msgs::PointCloud2>("pointcloud_msgs", 10);
//	ros::Subscriber pointcloud_sub = pointcloud_nh.subscribe("/camera/depth/color/points", 1, cb_pointcloud);
//
//	ros::spin();
//}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <heightmap_core/heightmap_core.hpp>
#include "heightmap_ros/heightmap_ros.hpp"

ros::Publisher pub; //

void pc_callback (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
{
//	float MIN_X = -0.5f;
//	float MAX_X = 0.5f;
//	float Z = 1.0f;
//	int DEPTH = 6;
//	camel::NodeBoundary boundary(MIN_X, MAX_X, Z);
//	camel::MapTreeNodeROS treeNode(boundary, DEPTH);

	// create DataNode and TreeNode
	camel::MapDataNodeROS dataNode;
	camel::MapTreeNodeROS treeNode(&dataNode);

	// convert for use XYZ
	sensor_msgs::PointCloud pointcloud_msg;
	sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud2_msg, pointcloud_msg);

	// get points
//	dataNode.FromMessage(pointcloud_msg);
	dataNode.FromMessagePointCloud2(*pointcloud2_msg);

	// sampling and rotation
	float rotationDegree = -45.0f;
	std::vector<camel::Point3*> samplingPoints = dataNode.SamplingPoints(5000);

	// insert points to tree and create 2.5d data
	treeNode.InsertMapTreeNode(samplingPoints);

	// declare for publish
	sensor_msgs::PointCloud output_pointcloud;
	sensor_msgs::PointCloud2 output_pointcloud2;

	// convert 2.5d data to sensor_msgs::Pointcloud2
	std::string fram_id = "d435_depth_optical_frame";
	dataNode.ToMessage(fram_id, output_pointcloud);
	sensor_msgs::convertPointCloudToPointCloud2(output_pointcloud, output_pointcloud2);

	// pub
	pub.publish(output_pointcloud2);

	std::cout << ros::Time::now() << std::endl;     // for check publish well
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pub_test");

	auto nh = ros::NodeHandle();
	pub = nh.advertise<sensor_msgs::PointCloud2>("heightmap2topic", 10);

	ros::Subscriber sub = nh.subscribe("/d435/depth/color/points", 1, pc_callback);

	ros::spin();
}