
#include "depalletizing_mapping_ros/MapTreeNodeROS.hpp"

namespace depalletizing_mapping
{
	MapTreeNodeROS::MapTreeNodeROS()
		: depalletizing_mapping::TreeNode()
	{
	}

	MapTreeNodeROS::MapTreeNodeROS(MapDataNodeROS* mapDataNode)
		: depalletizing_mapping::TreeNode(mapDataNode)
	{
	}

    MapTreeNodeROS::MapTreeNodeROS(MapDataNodeROS* mapDataNode, BoundingBox boundingBox)
        : depalletizing_mapping::TreeNode(mapDataNode, boundingBox)
    {
    }

	MapTreeNodeROS::MapTreeNodeROS(BoundingBox& boundingBox, int depth, int capacity)
		: depalletizing_mapping::TreeNode(boundingBox, depth, capacity)
	{
	}

	MapTreeNodeROS::MapTreeNodeROS(BoundingBox& boundingBox, int depth)
		: depalletizing_mapping::TreeNode(boundingBox, depth)
	{
	}

	MapTreeNodeROS::MapTreeNodeROS(BoundingBox& boundingBox)
		: depalletizing_mapping::TreeNode(boundingBox)
	{
	}

	MapTreeNodeROS::~MapTreeNodeROS()
	{
	}

}