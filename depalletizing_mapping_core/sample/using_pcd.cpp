//
// Created by wj on 22. 4. 19.
//
#include <iostream>
#include <heightmap_core/heightmap_core.hpp>

int main()
{
//	float X = 2.0f;
//	float MIN_Y = -1.0f;
//	float MAX_Y = 1.0f;
//	int DEPTH = 6;
//	camel::NodeBoundary boundary(X, MIN_Y, MAX_Y);
//	camel::MapTreeNode treeNode(boundary, DEPTH);

	// create DataNode and TreeNode
	camel::MapDataNode dataNode;				// for depalletizing_mapping
	camel::MapTreeNode treeNode(&dataNode);		// create quadtree with depalletizing_mapping address

	// get points from PCD file
	std::string inputPath = "/home/wj/Desktop/Data/input_data/stair_real.pcd";
	dataNode.FromPCD(inputPath);

	// sampling and rotate
	int samplingNum = 5000;
	float rotationDegree = -45.0f;
	std::vector<camel::Point3*> samplingPoints = dataNode.SamplingPointsWithRotate(samplingNum, rotationDegree);

	// insert points to tree and create 2.5d data
	treeNode.InsertMapTreeNode(samplingPoints);

	// write PCD file
	std::string outputPath = "/home/wj/Desktop/Data/output_data/";
	dataNode.ToPCD(outputPath);
}