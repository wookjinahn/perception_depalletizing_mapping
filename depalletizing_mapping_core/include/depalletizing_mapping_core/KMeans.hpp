//
// Created by wj on 22. 7. 25.
//

#ifndef DEPALLETIZING_MAPPING_CORE_KMEANS_HPP
#define DEPALLETIZING_MAPPING_CORE_KMEANS_HPP

#include <algorithm>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>

#include <depalletizing_mapping_core/Point3D.hpp>
#include "PlaneModel.hpp"

namespace depalletizing_mapping
{
	class KMeans
	{
	public:
		KMeans();
		KMeans(std::vector<Point3D> data, float k);

		std::vector<Point3D> GetData() const;
		void SetData(const std::vector<Point3D>& data);

        std::vector<Point3D> GetOutputData() const;
        void SetOutputData(const std::vector<Point3D>& outputData);
		Point2D GetOneCentroid() const;

		void FromPCD(const std::string& inputPath);
		void ToPCD(const std::vector<Point3D>& data, int num);
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

		std::vector<Point3D> mData;
        std::vector<Point3D> mOutputData;

		std::vector<Point2D> mCentroids;
		std::vector<PlaneModel> mModels;
	};
}

#endif //DEPALLETIZING_MAPPING_CORE_KMEANS_HPP
