
#include <ros/ros.h>
#include "depalletizing_demo/DepalletizingMappingDemo.hpp"

ros::Publisher pubHeightmap;
ros::Publisher pubPointCloud2;

void pc_callback (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
{
    const auto startTime = std::chrono::high_resolution_clock::now();
    // create DataNode and TreeNode
    depalletizing_mapping::PlaneModel planeModel;
    depalletizing_mapping::Ransac ransac(planeModel, 0.005f);
    depalletizing_mapping::MapDataNodeROS dataNode(ransac);
//    depalletizing_mapping::MapDataNodeROS dataNode;
    depalletizing_mapping::BoundingBox boundingBox(0, 0, 0.5, 0.5);
    depalletizing_mapping::MapTreeNodeROS treeNode(&dataNode, boundingBox);

    // convert for use XYZ
    sensor_msgs::PointCloud pointcloud_msg;
    sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud2_msg, pointcloud_msg);

    // get points
    dataNode.FromMessagePointCloudForDepalletizer(pointcloud_msg);
//    dataNode.FromMessagePointCloudForDepalletizer(pointcloud_msg);

    treeNode.InsertMapTreeNode();

    // declare for publish
    depalletizing_mapping_msgs::DepalletizingMap depalletizing_mapping_msgs;
    std::string depalletizing_mapping_frame_id = "base_frame";
    float cameraHeight = 1.32f;
    dataNode.ToHeightmapMsgsForDepalletizer(depalletizing_mapping_frame_id, depalletizing_mapping_msgs, cameraHeight);
//    dataNode.ToMessageHeightmapmsgs(depalletizing_mapping_frame_id, depalletizing_mapping_msgs, cameraHeight);
    pubHeightmap.publish(depalletizing_mapping_msgs);

    // pointcloud pub
    sensor_msgs::PointCloud output_pointcloud;
    sensor_msgs::PointCloud2 output_pointcloud2;
    std::string depalletizing_frame_id = "height_frame";
//    std::string depalletizing_frame_id = "base_link";

    std::vector<float> planerCenterPosition = dataNode.ToMessageForDepalletizerWithCenter(depalletizing_frame_id, output_pointcloud);

//    dataNode.ToMessageForDepalletizer(depalletizing_frame_id, output_pointcloud);
    pubPointCloud2.publish(output_pointcloud);

    const auto stopTime = std::chrono::high_resolution_clock::now();

    // add frame
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(planerCenterPosition[0], planerCenterPosition[1], planerCenterPosition[2]) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    brListener->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_position", "planar_frame"));

    const auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime);
    std::cout << elapsedTime.count() << " ms. (" << ((float)1000 / elapsedTime.count()) << " fps)" << std::endl;

    ros::Duration(0.5).sleep();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depalletizing_demo");

    auto pc_nh = ros::NodeHandle();
    auto odom_nh = ros::NodeHandle();
//    pubHeightmap = pc_nh.advertise<sensor_msgs::PointCloud2>("heightmap2topic", 10);
    pubHeightmap = pc_nh.advertise<depalletizing_mapping_msgs::DepalletizingMap>("heightmap_msgs", 10);
    pubPointCloud2 = pc_nh.advertise<sensor_msgs::PointCloud>("depalletizing_msgs", 10);

    ros::Subscriber sub = pc_nh.subscribe("/camera/depth/color/points", 1, pc_callback);

    ros::spin();
}