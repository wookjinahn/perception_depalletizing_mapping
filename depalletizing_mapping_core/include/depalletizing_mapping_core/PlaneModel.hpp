//
// Created by wj on 22. 7. 14.
//

#ifndef DEPALLETIZING_MAPPING_CORE_PLANEMODEL_HPP
#define DEPALLETIZING_MAPPING_CORE_PLANEMODEL_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <depalletizing_mapping_base/Point3D.hpp>

namespace depalletizing_mapping
{
	class PlaneModel
	{
	public:
        PlaneModel();
        PlaneModel(std::vector<Point3D>& data);
        PlaneModel(std::vector<Point3D>& data, std::vector<float>& parameter);

		std::vector<float> GetParameters() const;
		void SetModelThreshold(float modelThreshold);
		std::vector<Point3D> GetData() const;

		void FindParametersWithRandom(const std::vector<Point3D>& randomPoints);

		bool bIsInThreshold(const Point3D& data);
	private:
		std::vector<float> mParameters;	

		float mModelThreshold = 0.0f;

		std::vector<Point3D> mData;
	};
}

#endif //DEPALLETIZING_MAPPING_CORE_PLANEMODEL_HPP
