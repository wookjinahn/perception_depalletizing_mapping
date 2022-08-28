
#include "depalletizing_mapping_base/BoundingBox.hpp"

namespace depalletizing_mapping
{
	BoundingBox::BoundingBox()
		: mX(0.0f), mZ(0.0f), mW(1.0f), mH(1.0f)
	{
	}

	BoundingBox::BoundingBox(float x, float z, float w, float h)
		: mX(x)
		, mZ(z)
		, mW(w)
		, mH(h)
	{
	}

	float BoundingBox::GetX() const
	{
		return mX;
	}

	float BoundingBox::GetZ() const
	{
		return mZ;
	}

	float BoundingBox::GetW() const
	{
		return mW;
	}

	float BoundingBox::GetH() const
	{
		return mH;
	}

	void BoundingBox::SetBoundary(float x, float z, float w, float h)
	{
		mX = x;
		mZ = z;
		mW = w;
		mH = h;
	}

	bool BoundingBox::IsConstained(const Point3D& p) const
	{
		return (p.GetX() >= mX - mW && p.GetX() < mX + mW && p.GetZ() >= mZ - mH && p.GetZ() < mZ + mH);
	}
}