
#include <cmath>

#include "depalletizing_mapping_base/Point3D.hpp"

namespace depalletizing_mapping
{
    Point3D::Point3D()
            : mX(0.0f)
            , mY(0.0f)
            , mZ(0.0f)
    {
    }

    Point3D::Point3D(float x, float y, float z)
            : mX(x)
            , mY(y)
            , mZ(z)
    {
    }

    float Point3D::GetX() const
    {
        return mX;
    }

    float Point3D::GetY() const
    {
        return mY;
    }

    float Point3D::GetZ() const
    {
        return mZ;
    }

    void Point3D::SetX(float x)
    {
        mX = x;
    }

    void Point3D::SetY(float y)
    {
        mY = y;
    }

    void Point3D::SetZ(float z)
    {
        mZ = z;
    }

    void Point3D::SetXYZ(float x, float y, float z)
    {
        mX = x;
        mY = y;
        mZ = z;
    }

    bool Point3D::AscendingByX(Point3D& firstPoint, Point3D& secondPoint)
    {
        return firstPoint.GetX() < secondPoint.GetX();
    }

    bool Point3D::AscendingByY(Point3D& firstPoint, Point3D& secondPoint)
    {
        return firstPoint.GetY() < secondPoint.GetY();
    }

    bool Point3D::AscendingByZ(Point3D& firstPoint, Point3D& secondPoint)
    {
        return firstPoint.GetZ() < secondPoint.GetZ();
    }

    bool Point3D::DescendingByX(Point3D& firstPoint, Point3D& secondPoint)
    {
        return firstPoint.GetX() > secondPoint.GetX();
    }

    bool Point3D::DescendingByY(Point3D& firstPoint, Point3D& secondPoint)
    {
        return firstPoint.GetY() > secondPoint.GetY();
    }

    bool Point3D::DescendingByZ(Point3D& firstPoint, Point3D& secondPoint)
    {
        return firstPoint.GetZ() > secondPoint.GetZ();
    }

    bool Point3D::bIsEqual(const Point3D& other) const
    {
        if (mX == other.GetX() && mY == other.GetY() && mZ == other.GetZ())
        {
            return true;
        }
        return false;
    }

    float Point3D::DistanceBetweenOther(const Point3D& other) const
    {
        return std::sqrt((mX - other.GetX()) * (mX - other.GetX()) + (mY - other.GetY()) * (mY - other.GetY()) + (mZ - other.GetZ()) * (mZ - other.GetZ()));
    }
    Point2D Point3D::GetNodeKey() const
    {
        return mNodeKey;
    }

	Point2D Point3D::GetCentroid() const
	{
		return mCentroid;
	}

    void Point3D::SetNodeKey(const Point2D& nodeKey)
    {
		mNodeKey = nodeKey;
    }

    void Point3D::SetNodeKeyXZ(const float x, const float z)
    {
		mNodeKey.SetX(x);
		mNodeKey.SetZ(z);
    }

	void Point3D::SetCentroid(const Point2D& centroid)
	{
		mCentroid = centroid;
	}

	void Point3D::RotationRoll(float degree)
	{
		float rotationMatrix[3][3] = {{1.0f, 0.0f, 0.0f},
									  {0.0f, (float)std::cos(degree * D2R), (float)-std::sin(degree * D2R)},
									  {0.0f, (float)std::sin(degree * D2R), (float)std::cos(degree * D2R)}};

		float rotatedX = rotationMatrix[0][0] * this->GetX() + rotationMatrix[0][1] *  this->GetY() + rotationMatrix[0][2] *  this->GetZ();
		float rotatedY = rotationMatrix[1][0] * this->GetX() + rotationMatrix[1][1] *  this->GetY() + rotationMatrix[1][2] *  this->GetZ();
		float rotatedZ = rotationMatrix[2][0] * this->GetX() + rotationMatrix[2][1] *  this->GetY() + rotationMatrix[2][2] *  this->GetZ();
		this->SetXYZ(rotatedX, rotatedY, rotatedZ);
	}

	float Point3D::DistanceBetween2D(const Point2D& other)
	{
		return std::sqrt((GetX() - other.GetX()) * (GetX() - other.GetX()) + (GetZ() - other.GetZ()) * (GetZ() - other.GetZ()));
	}
}