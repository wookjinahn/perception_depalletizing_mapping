#ifndef DEPALLETIZING_MAPPING_BASE_TREENODE_HPP
#define DEPALLETIZING_MAPPING_BASE_TREENODE_HPP

#include <iostream>
#include <memory>	// unique_ptr

// STL
#include <algorithm>
#include <vector>

#include "BoundingBox.hpp"
#include "DataNode.hpp"
#include "Point2D.hpp"
#include "Point3D.hpp"

namespace depalletizing_mapping
{

class TreeNode
{
public:
	TreeNode();
	TreeNode(DataNode* mapDataNode);
    TreeNode(DataNode* mapDataNode, const BoundingBox& boundingBox);
	TreeNode(const BoundingBox& boundingBox, int depth, int capacity);
	TreeNode(const BoundingBox& boundingBox, int depth);
	TreeNode(const BoundingBox& boundingBox);
	~TreeNode();

	BoundingBox GetBoundary() const;
	DataNode* GetMapDataNode() const;

	void SetMapDataNode(DataNode* mapDataNode);
	void SetBoundary(BoundingBox& boundingBox);
	void SetBoundaryKey(float x, float z, float w, float h);
	void SetDepth(int depth);
	void SetCapacity(int capacity);

	void InsertMapTreeNode(std::vector<Point3D>& points);
    void InsertMapTreeNode();

private:
	void subdivideNode();
	void insertNodeRecursive(Point3D& point, DataNode* mapDataNode, int depth);

	DataNode* mMapDataNode = nullptr;
	BoundingBox mBoundindBox;
	int mDepth = 6;
	int mCapacity = 1;

	std::vector<Point3D> mCapacityPoints;

	bool mbDivided = false;
	std::unique_ptr<TreeNode> mNW = nullptr;
	std::unique_ptr<TreeNode> mNE = nullptr;
	std::unique_ptr<TreeNode> mSW = nullptr;
	std::unique_ptr<TreeNode> mSE = nullptr;
};

}

#endif //DEPALLETIZING_MAPPING_BASE_TREENODE_HPP
