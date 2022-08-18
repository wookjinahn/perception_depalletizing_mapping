//
// Created by wj on 22. 4. 19.
//

#ifndef DEPALLETIZING_MAPPING_ROS_MAPTREENODEROS_HPP
#define DEPALLETIZING_MAPPING_ROS_MAPTREENODEROS_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "depalletizing_mapping_core/depalletizing_mapping_core.hpp"
#include "depalletizing_mapping_ros/MapDataNodeROS.hpp"

namespace depalletizing_mapping
{

class MapTreeNodeROS : public depalletizing_mapping::TreeNode
{
public:
	MapTreeNodeROS();
	MapTreeNodeROS(MapDataNodeROS* mapDataNode);
    MapTreeNodeROS(MapDataNodeROS* mapDataNode, BoundingBox boundingBox);
	MapTreeNodeROS(BoundingBox& boundingBox, int depth, int capacity);
	MapTreeNodeROS(BoundingBox& boundingBox, int depth);
	MapTreeNodeROS(BoundingBox& boundingBox);
	~MapTreeNodeROS();
};

}


#endif //DEPALLETIZING_MAPPING_ROS_MAPTREENODEROS_HPP
