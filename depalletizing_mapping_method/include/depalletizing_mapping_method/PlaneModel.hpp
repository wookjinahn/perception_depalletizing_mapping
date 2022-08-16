//
// Created by wj on 22. 7. 14.
//

#ifndef DEPALLETIZING_MAPPING_METHOD_PLANEMODEL_HPP
#define DEPALLETIZING_MAPPING_METHOD_PLANEMODEL_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <heightmap_core/Point3.hpp>

namespace depalletizing_mapping
{
	class Plane
	{
	public:
		Plane();
		Plane(std::vector<camel::Point3>& data);
		Plane(std::vector<camel::Point3>& data, std::vector<float>& parameter);

		std::vector<float> GetParameters() const;
		void SetModelThreshold(float modelThreshold);
		std::vector<camel::Point3> GetData() const;

		void FindParametersWithRandom(const std::vector<camel::Point3>& randomPoints);

		bool bIsInThreshold(const camel::Point3& data);
	private:
		std::vector<float> mParameters;		// 여러개의 plane이 생길 경우도 생각.

		float mModelThreshold = 0.0f;

		std::vector<camel::Point3> mData;
	};
}

#endif //DEPALLETIZING_MAPPING_METHOD_PLANEMODEL_HPP
