//
// Created by wj on 22. 4. 19.
//

#include "depalletizing_mapping_core/MapTreeNode.hpp"

namespace depalletizing_mapping
{
	MapTreeNode::MapTreeNode()
	{
		MapDataNode heightmap;
		mMapDataNode = &heightmap;
		BoundingBox boundary(-1.0f, 1.0f, 2.0f);
        mBoundindBox = boundary;
		mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
		mMapDataNode->SetResolutionByBoundingBox(mDepth);
	}

	MapTreeNode::MapTreeNode(MapDataNode* mapDataNode)
		: mMapDataNode(mapDataNode)
	{
		BoundingBox boundary(-1.0f, 1.0f, 2.0f);
        mBoundindBox = boundary;
		mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
		mMapDataNode->SetResolutionByBoundingBox(mDepth);
	}

    MapTreeNode::MapTreeNode(MapDataNode* mapDataNode, const BoundingBox& boundingBox)
        : mMapDataNode(mapDataNode)
        , mBoundindBox(boundingBox)
    {
        mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
        mMapDataNode->SetResolutionByBoundingBox(mDepth);
    }

	MapTreeNode::MapTreeNode(const BoundingBox& boundingBox, int depth, int capacity)
		: mBoundindBox(boundingBox)
		, mDepth(depth)
		, mCapacity(capacity)
	{
		mCapacityPoints.reserve(mCapacity);
//        mMapDataNode->SetBoundingBox(mBoundindBox);
//        std::cout << "constructor End" << std::endl;
//		mMapDataNode->SetResolutionByBoundingBox(mDepth);
//        std::cout << "constructor End" << std::endl;
	}

	MapTreeNode::MapTreeNode(const BoundingBox& boundingBox, int depth)
		: mBoundindBox(boundingBox)
		, mDepth(depth)
	{
		mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
		mMapDataNode->SetResolutionByBoundingBox(mDepth);
	}

	MapTreeNode::MapTreeNode(const BoundingBox& boundingBox)
		: mBoundindBox(boundingBox)
		, mDepth(6)
	{
		mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
		mMapDataNode->SetResolutionByBoundingBox(mDepth);
	}

	MapTreeNode::~MapTreeNode()
	{
//		if (mMapDataNode != nullptr)
//		{
//			delete mMapDataNode;
//			mMapDataNode = nullptr;
//		}
	}

	BoundingBox MapTreeNode::GetBoundary() const
	{
		return mBoundindBox;
	}

	MapDataNode* MapTreeNode::GetMapDataNode() const
	{
		return mMapDataNode;
	}

    void MapTreeNode::SetMapDataNode(MapDataNode* mapDataNode)
    {
        if (!mMapDataNode)
        {
            mMapDataNode = mapDataNode;
        }
        else
        {
            delete mMapDataNode;
            mMapDataNode = mapDataNode;
        }
    }

	void MapTreeNode::SetBoundary(BoundingBox& boundingBox)
	{
        mBoundindBox = boundingBox;
	}

	void MapTreeNode::SetBoundaryKey(float x, float z, float w, float h)
	{
		mBoundindBox.SetBoundary(x, z, w, h);
	}

	void MapTreeNode::SetDepth(int depth)
	{
		mDepth = depth;
	}

	void MapTreeNode::SetCapacity(int capacity)
	{
		mCapacity = capacity;
		mCapacityPoints.reserve(mCapacity);
	}

	void MapTreeNode::InsertMapTreeNode(std::vector<Point3D>& points)
	{
		for (int i = 0; i < points.size(); i++)
		{
			int depth = 0;
			insertNodeRecursive(points[i], mMapDataNode, depth);
		}

        mMapDataNode->MakeMapToPoints();
	}

    void MapTreeNode::InsertMapTreeNode()
    {
        std::vector<Point3D> points = mMapDataNode->SamplingPoints(5000);

        for (int i = 0; i < points.size(); i++)
        {
            int depth = 0;
            insertNodeRecursive(points[i], mMapDataNode, depth);
        }

        mMapDataNode->MakeMapToPoints();
    }

	void MapTreeNode::subdivideNode()
	{
		float x = mBoundindBox.GetX();
		float z = mBoundindBox.GetZ();
		float w = mBoundindBox.GetW();
		float h = mBoundindBox.GetH();

		BoundingBox nw(x - w / 2, z + h / 2, w / 2, h / 2);
		BoundingBox ne(x + w / 2, z + h / 2, w / 2, h / 2);
		BoundingBox sw(x - w / 2, z - h / 2, w / 2, h / 2);
		BoundingBox se(x + w / 2, z - h / 2, w / 2, h / 2);

		mNW = std::make_unique<MapTreeNode>(nw, mDepth, mCapacity);
		mNE = std::make_unique<MapTreeNode>(ne, mDepth, mCapacity);
		mSW = std::make_unique<MapTreeNode>(sw, mDepth, mCapacity);
		mSE = std::make_unique<MapTreeNode>(se, mDepth, mCapacity);

		mbDivided = true;
	}

	void MapTreeNode::insertNodeRecursive(Point3D& point, MapDataNode* mapDataNode, int depth)
	{
		if (mDepth == depth)
		{
			point.SetNodeKeyXZ(mBoundindBox.GetX(), mBoundindBox.GetZ());
			mapDataNode->MakeHeightMap(point);
			return;
		}

		mCapacityPoints.push_back(point);

		if (mCapacity < mCapacityPoints.size() && mDepth > depth)
		{
			subdivideNode();
		}

		if (mbDivided)
		{
			while (!mCapacityPoints.empty())
			{
				Point3D qPoint = mCapacityPoints.back();
				mCapacityPoints.pop_back();

				if (mNW->mBoundindBox.IsConstained(qPoint))
				{
					mNW->insertNodeRecursive(qPoint, mapDataNode, ++depth);
				}
				else if (mNE->mBoundindBox.IsConstained(qPoint))
				{
					mNE->insertNodeRecursive(qPoint, mapDataNode, ++depth);
				}
				else if (mSW->mBoundindBox.IsConstained(qPoint))
				{
					mSW->insertNodeRecursive(qPoint, mapDataNode, ++depth);
				}
				else if (mSE->mBoundindBox.IsConstained(qPoint))
				{
					mSE->insertNodeRecursive(qPoint, mapDataNode, ++depth);
				}
			}
		}
	}
}