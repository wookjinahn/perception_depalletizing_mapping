#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>    // subscribe d435 pointcloud
#include <tf/transform_broadcaster.h>   // broadcast tf detected planar region center point

#include <depalletizing_mapping_ros/depalletizing_mapping_ros.hpp>      // depalletizing_mapping

ros::Publisher pubDepalletizingMap;
ros::Publisher pubPointCloud2;
ros::Publisher pubPolygon;
tf::TransformBroadcaster* brListener;

void depalletizing_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
{
    // for check runtime
    const auto startTime = std::chrono::high_resolution_clock::now();

    // create DataNode and TreeNode
    depalletizing_mapping::MapDataNodeROS dataNode;
    depalletizing_mapping::MapTreeNodeROS treeNode(&dataNode);

    // set camera position height
    float cameraHeight = 1.32f;     // 1.32 m
    dataNode.SetCameraHeight(cameraHeight);

    // get raw data
    dataNode.FromPointCloud2Msgs(*pointcloud2_msg);

    // process data
    treeNode.RunTreeProcess();
    dataNode.DetectPlanarRegion();

    // depalletizing_map publish
    depalletizing_mapping_msgs::DepalletizingMap depalletizing_mapping_msgs;
    std::string depalletizing_mapping_frame_id = "depalletizing_base_frame";
    dataNode.ToDepalletizingMapMsgs(depalletizing_mapping_frame_id, depalletizing_mapping_msgs);
    pubDepalletizingMap.publish(depalletizing_mapping_msgs);

    // detected planar region pointcloud publish
    sensor_msgs::PointCloud2 planar_points;
    std::string planar_points_frame_id = "depalletizing_frame";
    std::vector<float> planerCenterPosition = dataNode.ToPointCloud2Msgs(planar_points_frame_id, planar_points);
    pubPointCloud2.publish(planar_points);

    // detected planar region planar_polygon publish
    geometry_msgs::PolygonStamped planar_polygon;
    std::string planar_polygon_frame_id = "depalletizing_frame";
    dataNode.ToPolygonMsgs(planar_polygon_frame_id, planar_polygon);
    pubPolygon.publish(planar_polygon);

    // add frame about detected planar region center position
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(planerCenterPosition[0], planerCenterPosition[1], planerCenterPosition[2]));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    brListener->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_position", "planar_frame"));

    // check runtime
    const auto stopTime = std::chrono::high_resolution_clock::now();
    const auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime);
    std::cout << "process runtime : " << elapsedTime.count() << " ms. \t";

    // thread sleep set
    float sleepTime = 0.5;
    ros::Duration(sleepTime).sleep();
    std::cout << "ros sleep : " << sleepTime << " s." << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depalletizing_demo");

    auto pc_nh = ros::NodeHandle();
    pubDepalletizingMap = pc_nh.advertise<depalletizing_mapping_msgs::DepalletizingMap>("depalletizing_msgs", 10);
    pubPointCloud2 = pc_nh.advertise<sensor_msgs::PointCloud2>("depalletizing_planar_points", 10);
    pubPolygon = pc_nh.advertise<geometry_msgs::PolygonStamped>("depalletizing_planar_polygon", 10);

    tf::TransformBroadcaster br;
    brListener = &br;

    ros::Subscriber sub = pc_nh.subscribe("/camera/depth/color/points", 1, depalletizing_callback);

    ros::spin();
}