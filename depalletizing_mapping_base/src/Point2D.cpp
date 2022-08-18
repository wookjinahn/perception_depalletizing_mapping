
#include <cmath>

#include "depalletizing_mapping_base/Point2D.hpp"

namespace depalletizing_mapping
{
    Point2D::Point2D()
            : mX(0)
            , mZ(0)
    {
    }

    Point2D::Point2D(float x, float z)
            : mX(x)
            , mZ(z)
    {
    }

    float Point2D::GetX() const
    {
        return mX;
    }

    float Point2D::GetZ() const
    {
        return mZ;
    }

    void Point2D::SetX(float x)
    {
        mX = x;
    }

    void Point2D::SetZ(float z)
    {
        mZ = z;
    }
    bool Point2D::AscendingByX(Point2D& firstPoint, Point2D& secondPoint)
    {
        return firstPoint.GetX() < secondPoint.GetX();
    }

    bool Point2D::AscendingByZ(Point2D& firstPoint, Point2D& secondPoint)
    {
        return firstPoint.GetZ() < secondPoint.GetZ();
    }
    bool Point2D::DescendingByX(Point2D& firstPoint, Point2D& secondPoint)
    {
        return firstPoint.GetX() > secondPoint.GetX();
    }

    bool Point2D::DescendingByZ(Point2D& firstPoint, Point2D& secondPoint)
    {
        return firstPoint.GetZ() > secondPoint.GetZ();
    }

    bool Point2D::bIsEqual(const Point2D& other) const
    {
        if (mX == other.GetX() && mZ == other.GetZ())
        {
            return true;
        }
        return false;
    }

    float Point2D::DistanceBetweenOther(const Point2D& other) const
    {
        return std::sqrt((mX - other.GetX()) * (mX - other.GetX()) + (mZ - other.GetZ()) * (mZ - other.GetZ()));
    }

}