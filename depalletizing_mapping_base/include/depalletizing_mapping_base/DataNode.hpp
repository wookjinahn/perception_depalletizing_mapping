#ifndef DEPALLETIZING_MAPPING_BASE_DATANODE_HPP
#define DEPALLETIZING_MAPPING_BASE_DATANODE_HPP

#include <fstream>		// file oi (std::ifstream)
#include <iostream>
#include <random>		// sampling
#include <sstream>		// file oi (std::istringstream)
#include <string>		// data path

// STL
#include <algorithm> 	// std::shuffle
#include <map>
#include <unordered_map>
#include <vector>

#include "BoundingBox.hpp"
#include "Point3D.hpp"

namespace depalletizing_mapping
{

struct pair_hash
{
	template <class T1, class T2>
	std::size_t operator() (const std::pair<T1, T2> &pair) const
	{
		return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
	}
};

class DataNode
{
public:
	DataNode();
	~DataNode();

	double GetResolution() const;
	void SetResolution(double resolution);
	void SetResolutionByBoundingBox(const BoundingBox& boundingBox, int depth);
    void SetResolutionByBoundingBox(int depth);

    BoundingBox GetBoundingBox() const;
    void SetBoundingBox(BoundingBox boundingBox);

    std::unordered_map<std::pair<float, float>, float, pair_hash> GetMapDataPair() const;

	std::vector<Point3D> GetInputPoints() const;
    void SetInputPoint(Point3D point);

	std::vector<Point3D> GetOutputPoints() const;
    void SetOutputPoint(const std::vector<Point3D>& outputPoint);

	std::vector<Point3D> SamplingPointsWithRotate(int samplingNum, float rotationDegree);
	std::vector<Point3D> SamplingPoints(int samplingNum);
	void MakeHeightMap(Point3D& point);
	void MakeMapToPoints();		// at ToPCD

	void FromPCD(const std::string &inputPath);
	void ToPCD(const std::string &outputPath);

protected:
    double mResolution;

private:
    BoundingBox mBoundingBox;

	std::unordered_map<std::pair<float, float>, float, pair_hash> mMapDataPair;

	std::vector<Point3D> mInputPoints;
	std::vector<Point3D> mOutputPoints;
};

}

#endif //DEPALLETIZING_MAPPING_BASE_DATANODE_HPP
