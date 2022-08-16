
#include "depalletizing_mapping_ros/MapTreeNodeROS.hpp"

namespace depalletizing_mapping
{
	MapTreeNodeROS::MapTreeNodeROS()
		: depalletizing_mapping::MapTreeNode()
	{
	}

	MapTreeNodeROS::MapTreeNodeROS(MapDataNodeROS* mapDataNode)
		: depalletizing_mapping::MapTreeNode(mapDataNode)
	{
	}

    MapTreeNodeROS::MapTreeNodeROS(MapDataNodeROS* mapDataNode, BoundingBox boundingBox)
        : depalletizing_mapping::MapTreeNode(mapDataNode, boundingBox)
    {
    }

	MapTreeNodeROS::MapTreeNodeROS(BoundingBox& boundingBox, int depth, int capacity)
		: depalletizing_mapping::MapTreeNode(boundingBox, depth, capacity)
	{
	}

	MapTreeNodeROS::MapTreeNodeROS(BoundingBox& boundingBox, int depth)
		: depalletizing_mapping::MapTreeNode(boundingBox, depth)
	{
	}

	MapTreeNodeROS::MapTreeNodeROS(BoundingBox& boundingBox)
		: depalletizing_mapping::MapTreeNode(boundingBox)
	{
	}

	MapTreeNodeROS::~MapTreeNodeROS()
	{
	}

}