#ifndef DEPALLETIZING_MAPPING_ROS_MAPDATANODEROS_HPP
#define DEPALLETIZING_MAPPING_ROS_MAPDATANODEROS_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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

	void SetRotateDegree(double rotateDegree);
	double GetRotateDegree() const;
	void SetRotateRadian(double rotateRadian);
	double GetRotateRadian() const;

    void FromMessage(sensor_msgs::PointCloud pointcloud_msg);
	void ToMessage(std::string frame_id, sensor_msgs::PointCloud& output_pointcloud);
	void FromMessagePointCloud2(sensor_msgs::PointCloud2 pointcloud2_msgs);
	void ToMessagePointCloud2(std::string frame_id, sensor_msgs::PointCloud2& output_pointcloud2);
	void ToMessageHeightmapmsgs(std::string frame_id, depalletizing_mapping_msgs::DepalletizingMapConstPtr& output_msgs);
	void ToPointCloud2Msgs();
	void fromPointCloud2Msgs();

    // depalletizing
    void ToPCDForDepalletizer(const std::string& outputPath);
    void FromMessagePointCloud2ForDepalletizer(sensor_msgs::PointCloud2 pointcloud2_msgs);
    void FromMessagePointCloudForDepalletizer(sensor_msgs::PointCloud pointcloud_msgs);
    void ToMessageForDepalletizer(std::string frame_id, sensor_msgs::PointCloud& output_pointcloud);
    std::vector<float> ToMessageForDepalletizerWithCenter(std::string frame_id, sensor_msgs::PointCloud& output_pointcloud);
    void ToHeightmapMsgsForDepalletizer(std::string frame_id, depalletizing_mapping_msgs::DepalletizingMap& depalletizing_mapping_msgs, float cameraHeight);

private:

    std::unordered_map<std::pair<float, float>, float, pair_hash> mOldMapData;

    std::vector<Point3D> mPastData;
    std::vector<Point3D> mOutputPoints;
	float mRotateDegree;
    BoundingBox mPositionRange;
};

}

#endif //DEPALLETIZING_MAPPING_ROS_MAPDATANODEROS_HPP
