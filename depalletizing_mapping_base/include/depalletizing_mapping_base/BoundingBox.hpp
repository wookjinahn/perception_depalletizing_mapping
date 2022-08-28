#ifndef DEPALLETIZING_MAPPING_BASE_BOUNDINGBOX_HPP
#define DEPALLETIZING_MAPPING_BASE_BOUNDINGBOX_HPP

#include "Point3D.hpp"

namespace depalletizing_mapping
{

class BoundingBox
{
public:
	BoundingBox();
	BoundingBox(float x, float z, float w, float h);

	float GetX() const;
	float GetZ() const;
	float GetW() const;
	float GetH() const;

	void SetBoundary(float x, float z, float w, float h);

	bool IsConstained(const Point3D& p) const;

private:
	float mX, mZ, mW, mH;
};

}

#endif //DEPALLETIZING_MAPPING_BASE_BOUNDINGBOX_HPP
