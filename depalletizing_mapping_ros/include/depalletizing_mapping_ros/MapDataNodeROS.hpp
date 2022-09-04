#ifndef DEPALLETIZING_MAPPING_ROS_MAPDATANODEROS_HPP
#define DEPALLETIZING_MAPPING_ROS_MAPDATANODEROS_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <depalletizing_mapping_base/depalletizing_mapping_base.hpp>
#include <depalletizing_mapping_core/depalletizing_mapping_core.hpp>
#include <depalletizing_mapping_msgs/DepalletizingMap.h>

namespace depalletizing_mapping
{

class MapDataNodeROS : public depalletizing_mapping::MapDataNode
{
public:
	MapDataNodeROS();
    MapDataNodeROS(Ransac& ransac);
	~MapDataNodeROS();

    void SetCameraHeight(float cameraHeight);
    float GetCameraHeight() const;

    // depalletizing
    void FromPointCloud2Msgs(sensor_msgs::PointCloud2 pointcloud2_msgs);
    std::vector<float> ToPointCloud2Msgs(std::string frame_id, sensor_msgs::PointCloud2& output_pointcloud);
    void ToDepalletizingMapMsgs(std::string frame_id, depalletizing_mapping_msgs::DepalletizingMap& depalletizing_mapping_msgs);
    void ToPolygonMsgs(std::string frame_id, geometry_msgs::PolygonStamped& polygon_msgs);

private:
    float mCameraHeight;
};

}

#endif //DEPALLETIZING_MAPPING_ROS_MAPDATANODEROS_HPP
