//
// Created by wj on 22. 7. 25.
//

#ifndef DEPALLETIZING_MAPPING_METHOD_KMEANS_HPP
#define DEPALLETIZING_MAPPING_METHOD_KMEANS_HPP

#include <algorithm>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>

#include <heightmap_core/Point3.hpp>
//#include "Point3.hpp"
#include "PlaneModel.hpp"

namespace depalletizing_mapping
{
	class KMeans
	{
	public:
		KMeans();
		KMeans(std::vector<Point3> data, float k);

		std::vector<Point3> GetData() const;
		void SetData(const std::vector<Point3>& data);

        std::vector<Point3> GetOutputData() const;
        void SetOutputData(const std::vector<Point3>& outputData);
		camel::Point2 GetOneCentroid() const;

		void FromPCD(const std::string& inputPath);
		void ToPCD(const std::vector<camel::Point3>& data, int num);
		void Run();
		void SaveResult();
        void SaveOneResult();

	private:
		void setInitialCentroid();
		void assignCentroid();
		bool bUpdateCentroid();
		bool bFindNearRate(int nearNumParameter);
		void generateCentroid();
		void clustering();

		std::vector<Point3> mData;
        std::vector<Point3> mOutputData;

		std::vector<Point2> mCentroids;
		std::vector<Model::Plane> mModels;
	};
}

#endif //DEPALLETIZING_MAPPING_METHOD_KMEANS_HPP
