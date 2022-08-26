//
// Created by wj on 22. 7. 15.
//

#ifndef DEPALLETIZING_MAPPING_CORE_RANSAC_HPP
#define DEPALLETIZING_MAPPING_CORE_RANSAC_HPP

#include <iostream>
#include <algorithm>
#include <random>

#include <depalletizing_mapping_base/Point3D.hpp>

#include "PlaneModel.hpp"

namespace depalletizing_mapping
{
//	template<typename Model, typename camelVector>
	class Ransac
	{
	public:
        Ransac();
		Ransac(PlaneModel& model, std::vector<Point3D>& data, float modelThreshold, int maxIteration);
        Ransac(PlaneModel& model, float modelThreshold, int maxIteration);
        Ransac(PlaneModel& model, float modelThreshold);
        Ransac(float modelThreshold);

		void SetData(std::vector<Point3D>& data);
        void SetMaxIteration (int maxiteration);

		std::vector<Point3D> GetData() const;
		std::vector<float> GetBestModelParameters() const;
		std::vector<PlaneModel> GetResultModel() const;
		std::vector<Point3D> GetResultData() const;

		bool bRun();
		void RunMulti();

		void GetResult();
		void GetResultMulti();

	private:
		std::vector<Point3D> getUpperPoints(int number) const;
		std::vector<Point3D> getRandomPoints() const;
		std::vector<Point3D> getRandomPoints(std::vector<Point3D>& upperPoints) const;

		int getInlierNum();
		int getInlierNum(std::vector<Point3D>& upperPoints);

		bool bIsContained(Point3D& data);
		bool bIsContainedMulit(Point3D& data, std::vector<float>& parameters);

		PlaneModel mModel;
		std::vector<Point3D> mData;
		std::vector<Point3D> mResultData;
		std::vector<PlaneModel> mResultModel;

		int mMaxIteration = 0;
		float mModelThreshold = 0;
		std::vector<float> mBestModelParameters;
		int mInlierNum = 0;
	};
}


#endif //DEPALLETIZING_MAPPING_CORE_RANSAC_HPP
