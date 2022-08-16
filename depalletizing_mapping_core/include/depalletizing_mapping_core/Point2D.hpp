#ifndef DEPALLETIZING_MAPPING_CORE_POINT2D_HPP
#define DEPALLETIZING_MAPPING_CORE_POINT2D_HPP

namespace depalletizing_mapping
{

class Point2D
{
public:
    Point2D();
    Point2D(float x, float z);

    float GetX() const;
    float GetZ() const;

    void SetX(float x);
    void SetZ(float z);

    static bool AscendingByX(Point2D& firstPoint, Point2D& secondPoint);
    static bool AscendingByZ(Point2D& firstPoint, Point2D& secondPoint);
    static bool DescendingByX(Point2D& firstPoint, Point2D& secondPoint);
    static bool DescendingByZ(Point2D& firstPoint, Point2D& secondPoint);

    bool bIsEqual(const Point2D& other) const;

    float DistanceBetweenOther(const Point2D& other) const;

private:
    float mX, mZ;
};

}

#endif //DEPALLETIZING_MAPPING_CORE_POINT2D_HPP
