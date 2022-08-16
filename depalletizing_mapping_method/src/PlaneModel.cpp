
#include "depalletizing_mapping_method/PlaneModel.hpp"

namespace depalletizing_mapping
{
	Plane::Plane()
	{
	}

	Plane::Plane(std::vector<camel::Point3>& data)
	{
		mData = data;
	}

	Plane::Plane(std::vector<camel::Point3>& data, std::vector<float>& parameter)
	{
		mData = data;
		mParameters = parameter;
	}

	std::vector<float> Plane::GetParameters() const
	{
		return mParameters;
	}

	void Plane::SetModelThreshold(float modelThreshold)
	{
		mModelThreshold = modelThreshold;
	}


	std::vector<camel::Point3> Plane::GetData() const
	{
		return mData;
	}

	void Plane::FindParametersWithRandom(const std::vector<camel::Point3>& randomPoints)
	{
		mParameters.clear();

		float x1 = randomPoints[0].GetX();
		float y1 = randomPoints[0].GetY();
		float z1 = randomPoints[0].GetZ();
		float x2 = randomPoints[1].GetX();
		float y2 = randomPoints[1].GetY();
		float z2 = randomPoints[1].GetZ();
		float x3 = randomPoints[2].GetX();
		float y3 = randomPoints[2].GetY();
		float z3 = randomPoints[2].GetZ();

		float det_A = x1 * (y2 * z3 - y3 * z2) - x2 * (y1 * z3 - y3 * z1) + x3 * (y1 * z2 - y2 * z1);

		float adj_A00 = y2 * z3 - y3 * z2;
		float adj_A01 = -(y1 * z3 - y3 * z1);
		float adj_A02 = y1 * z2 - y2 * z1;
		float adj_A10 = -(x2 * z3 - x3 * z2);
		float adj_A11 = x1 * z3 - x3 * z1;
		float adj_A12 = -(x1 * z2 - x2 * z1);
		float adj_A20 = x2 * y3 - x3 * y2;
		float adj_A21 = -(x1 * y3 - x3 * y1);
		float adj_A22 = x1 * y2 - x2 * y1;

		float parameterA = (adj_A00 + adj_A01 + adj_A02) / det_A;
		float parameterB = (adj_A10 + adj_A11 + adj_A12) / det_A;
		float parameterC = (adj_A20 + adj_A21 + adj_A22) / det_A;

		mParameters = { parameterA, parameterB, parameterC };
	}

	bool Plane::bIsInThreshold(const camel::Point3& data)
	{
		float x = data.GetX();
		float y = data.GetY();
		float z = data.GetZ();

		float distance = std::abs(mParameters[0] * x + mParameters[1] * y + mParameters[2] * z - 1) / std::sqrt(mParameters[0] * mParameters[0] + mParameters[1] * mParameters[1] + mParameters[2] * mParameters[2]);

		if (distance < mModelThreshold)
		{
			return true;
		}
		return false;
	}
}