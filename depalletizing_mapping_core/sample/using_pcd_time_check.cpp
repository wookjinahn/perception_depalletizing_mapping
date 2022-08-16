//
// Created by wj on 22. 4. 19.
//
#include <iostream>
#include <heightmap_core/heightmap_core.hpp>

int main()
{
    clock_t start;
    clock_t end;

    float X = 2.0f;
    float MIN_Y = -1.0f;
    float MAX_Y = 1.0f;
    int DEPTH = 6;
    camel::NodeBoundary boundary(X, MIN_Y, MAX_Y);

	clock_t start;
	clock_t end;

	camel::MapDataNode dataNode;				// for depalletizing_mapping
	camel::MapTreeNode treeNode(&dataNode);		// create quadtree with depalletizing_mapping address

	// ----- Read PCD file -----
	std::cout << "Read PCD File : ";

	std::string inputPath = "/home/wj/Desktop/Data/input_data/stair_real.pcd";
	start = clock();

	dataNode.FromPCD(inputPath);

	end = clock();
	std::cout << ((double)(end - start)) / (long)CLOCKS_PER_SEC << " sec" << std::endl;

	// ----- sampling -----
	std::cout << "Sampling : ";
	start = clock();

	int samplingNum = 5000;
	float ratationDegree = -45.0f
	std::vector<camel::Point3*> samplingPoints = dataNode.SamplingPointsWithRotate(samplingNum, ratationDegree);

	end = clock();
	std::cout << ((double)(end - start)) / (long)CLOCKS_PER_SEC << " sec" << std::endl;

	// ----- Insert in Quadtree -----
	std::cout << "Insert in Quadtree : ";

	start = clock();

	treeNode.InsertMapTreeNode(samplingPoints);

	end = clock();
	std::cout << ((double)(end - start)) / (long)CLOCKS_PER_SEC << " sec" << std::endl;

	// ----- Write PCD file -----
	std::cout << "Write PCD File : ";
	start = clock();

	std::string outputPath = "/home/wj/Desktop/Data/output_data/";

	dataNode.ToPCD(outputPath);
	end = clock();
	std::cout << ((double)(end - start)) / (long)CLOCKS_PER_SEC << " sec" << std::endl;
}