//
// Created by wj on 22. 4. 19.
//

#include "depalletizing_mapping_base/TreeNode.hpp"

namespace depalletizing_mapping
{
	TreeNode::TreeNode()
	{
		DataNode heightmap;
		mMapDataNode = &heightmap;
		BoundingBox boundary(0, 0, 0.5, 0.5);
        mBoundindBox = boundary;
		mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
		mMapDataNode->SetResolutionByBoundingBox(mDepth);
	}

	TreeNode::TreeNode(DataNode* mapDataNode)
		: mMapDataNode(mapDataNode)
	{
		BoundingBox boundary(0, 0, 0.5, 0.5);
        mBoundindBox = boundary;
		mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
		mMapDataNode->SetResolutionByBoundingBox(mDepth);
	}

    TreeNode::TreeNode(DataNode* mapDataNode, const BoundingBox& boundingBox)
        : mMapDataNode(mapDataNode)
        , mBoundindBox(boundingBox)
    {
        mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
        mMapDataNode->SetResolutionByBoundingBox(mDepth);
    }

	TreeNode::TreeNode(const BoundingBox& boundingBox, int depth, int capacity)
		: mBoundindBox(boundingBox)
		, mDepth(depth)
		, mCapacity(capacity)
	{
		mCapacityPoints.reserve(mCapacity);
	}

	TreeNode::TreeNode(const BoundingBox& boundingBox, int depth)
		: mBoundindBox(boundingBox)
		, mDepth(depth)
	{
		mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
		mMapDataNode->SetResolutionByBoundingBox(mDepth);
	}

	TreeNode::TreeNode(const BoundingBox& boundingBox)
		: mBoundindBox(boundingBox)
		, mDepth(6)
	{
		mCapacityPoints.reserve(mCapacity);
        mMapDataNode->SetBoundingBox(mBoundindBox);
		mMapDataNode->SetResolutionByBoundingBox(mDepth);
	}

	TreeNode::~TreeNode()
	{
	}

	BoundingBox TreeNode::GetBoundary() const
	{
		return mBoundindBox;
	}

	DataNode* TreeNode::GetMapDataNode() const
	{
		return mMapDataNode;
	}

    void TreeNode::SetMapDataNode(DataNode* mapDataNode)
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

	void TreeNode::SetBoundary(BoundingBox& boundingBox)
	{
        mBoundindBox = boundingBox;
	}

	void TreeNode::SetBoundaryKey(float x, float z, float w, float h)
	{
		mBoundindBox.SetBoundary(x, z, w, h);
	}

	void TreeNode::SetDepth(int depth)
	{
		mDepth = depth;
		mMapDataNode->SetResolutionByBoundingBox(mDepth);
	}

	void TreeNode::SetCapacity(int capacity)
	{
		mCapacity = capacity;
		mCapacityPoints.reserve(mCapacity);
	}

	void TreeNode::RunTreeProcess(std::vector<Point3D>& points)
	{
        insertTreeNode(points);
	}

    void TreeNode::RunTreeProcess()
    {
        insertTreeNode();
    }

    void TreeNode::insertTreeNode(std::vector<Point3D>& points)
    {
        for (int i = 0; i < points.size(); i++)
        {
            int depth = 0;
            insertNodeRecursive(points[i], mMapDataNode, depth);
        }

        mMapDataNode->MakeMapToPoints();
    }

    void TreeNode::insertTreeNode()
    {
        std::vector<Point3D> points = mMapDataNode->SamplingPoints(100000);

        for (int i = 0; i < points.size(); i++)
        {
            int depth = 0;
            insertNodeRecursive(points[i], mMapDataNode, depth);
        }

        mMapDataNode->MakeMapToPoints();
    }

	void TreeNode::subdivideNode()
	{
		float x = mBoundindBox.GetX();
		float z = mBoundindBox.GetZ();
		float w = mBoundindBox.GetW();
		float h = mBoundindBox.GetH();

		BoundingBox nw(x - w / 2, z + h / 2, w / 2, h / 2);
		BoundingBox ne(x + w / 2, z + h / 2, w / 2, h / 2);
		BoundingBox sw(x - w / 2, z - h / 2, w / 2, h / 2);
		BoundingBox se(x + w / 2, z - h / 2, w / 2, h / 2);

		mNW = std::make_unique<TreeNode>(nw, mDepth, mCapacity);
		mNE = std::make_unique<TreeNode>(ne, mDepth, mCapacity);
		mSW = std::make_unique<TreeNode>(sw, mDepth, mCapacity);
		mSE = std::make_unique<TreeNode>(se, mDepth, mCapacity);

		mbDivided = true;
	}

	void TreeNode::insertNodeRecursive(Point3D& point, DataNode* mapDataNode, int depth)
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