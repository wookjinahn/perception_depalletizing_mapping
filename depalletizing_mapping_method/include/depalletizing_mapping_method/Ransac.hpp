//
// Created by wj on 22. 7. 15.
//

#ifndef DEPALLETIZING_MAPPING_METHOD_RANSAC_HPP
#define DEPALLETIZING_MAPPING_METHOD_RANSAC_HPP

#include <iostream>
#include <algorithm>
#include <random>

#include <heightmap_core/Point3.hpp>

#include "PlaneModel.hpp"

namespace depalletizing_mapping
{
//	template<typename Model, typename camelVector>
	class Ransac
	{
	public:
		Ransac(Model::Plane& model, std::vector<camel::Point3>& data, float modelThreshold, int maxIteration);
        Ransac(Model::Plane& model, float modelThreshold, int maxIteration);
        Ransac(Model::Plane& model, float modelThreshold);

		void SetData(std::vector<camel::Point3>& data);
        void SetMaxIteration (int maxiteration);

		std::vector<camel::Point3> GetData() const;
		std::vector<float> GetBestModelParameters() const;
		std::vector<Model::Plane> GetResultModel() const;
		std::vector<camel::Point3> GetResultData() const;

		bool bRun();
		void RunMulti();

		void GetResult();
		void GetResultMulti();

	private:
		std::vector<camel::Point3> getUpperPoints(int number) const;
		std::vector<camel::Point3> getRandomPoints() const;
		std::vector<camel::Point3> getRandomPoints(std::vector<camel::Point3>& upperPoints) const;

		int getInlierNum();
		int getInlierNum(std::vector<camel::Point3>& upperPoints);

		bool bIsContained(camel::Point3& data);
		bool bIsContainedMulit(camel::Point3& data, std::vector<float>& parameters);

		Model::Plane mModel;
		std::vector<camel::Point3> mData;
		std::vector<camel::Point3> mResultData;
		std::vector<Model::Plane> mResultModel;

		int mMaxIteration = 0;
		float mModelThreshold = 0;
		std::vector<float> mBestModelParameters;
		int mInlierNum = 0;
	};
}


#endif //DEPALLETIZING_MAPPING_METHOD_RANSAC_HPP
