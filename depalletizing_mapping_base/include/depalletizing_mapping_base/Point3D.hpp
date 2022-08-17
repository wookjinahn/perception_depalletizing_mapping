#ifndef DEPALLETIZING_MAPPING_BASE_POINT3D_HPP
#define DEPALLETIZING_MAPPING_BASE_POINT3D_HPP

#include "Point2D.hpp"

const double PI = 3.14159265359;
const double D2R = PI / 180;
const double R2D = 180 / PI;

namespace depalletizing_mapping
{

class Point3D
{
public:
	Point3D();
	Point3D(float x, float y, float z);

    float GetX() const;
    float GetY() const;
    float GetZ() const;

    void SetX(float x);
    void SetY(float y);
    void SetZ(float z);
    void SetXYZ(float x, float y, float z) ;

    // sorting
    static bool AscendingByX(Point3D& firstPoint, Point3D& secondPoint);
    static bool AscendingByY(Point3D& firstPoint, Point3D& secondPoint);
    static bool AscendingByZ(Point3D& firstPoint, Point3D& secondPoint);
    static bool DescendingByX(Point3D& firstPoint, Point3D& secondPoint);
    static bool DescendingByY(Point3D& firstPoint, Point3D& secondPoint);
    static bool DescendingByZ(Point3D& firstPoint, Point3D& secondPoint);

    bool bIsEqual(const Point3D& other) const;

    float DistanceBetweenOther(const Point3D& other) const;

	Point2D GetNodeKey() const;
	Point2D GetCentroid() const;

	void SetNodeKey(const Point2D& nodeKey);
	void SetNodeKeyXZ(float const x, float const z);
	void SetCentroid(const Point2D& centroid);

	void RotationRoll(float degree);

	float DistanceBetween2D(const Point2D& other);

private:
    float mX, mY, mZ;
	Point2D mNodeKey;
	Point2D mCentroid;
};

}

#endif //DEPALLETIZING_MAPPING_BASE_POINT3D_HPP
