#ifndef DEPALLETIZING_MAPPING_CORE_QUICKHULL_HPP
#define DEPALLETIZING_MAPPING_CORE_QUICKHULL_HPP

#include <algorithm>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>

#include <depalletizing_mapping_base/Point3D.hpp>
#include "PlaneModel.hpp"

namespace depalletizing_mapping
{
    class QuickHull
    {
    public:
        QuickHull();
        QuickHull(std::vector<depalletizing_mapping::Point3D>& inputPoint);

        void SetInputData(std::vector<depalletizing_mapping::Point3D>& inputPoint);
        std::vector<depalletizing_mapping::Point3D> GetInputData() const;

        void SetOutputData(std::vector<depalletizing_mapping::Point3D>& outPoint);
        std::vector<depalletizing_mapping::Point3D> GetOutputData() const;

        void Run();

    private:
        int findSide(depalletizing_mapping::Point3D point1, depalletizing_mapping::Point3D point2, depalletizing_mapping::Point3D point);
        float lineDist(depalletizing_mapping::Point3D point1, depalletizing_mapping::Point3D point2, depalletizing_mapping::Point3D point);
        void quickHull(const std::vector<depalletizing_mapping::Point3D>& data, depalletizing_mapping::Point3D point1, depalletizing_mapping::Point3D point2, int side);

//        static bool sameValue(depalletizing_mapping::Point3D point);
        void getAngleEachPoints();
        static bool DescendingByAngle(depalletizing_mapping::Point3D& firstPoint, depalletizing_mapping::Point3D& secondPoint);
        void sortByAngle();

        std::vector<depalletizing_mapping::Point3D> mInputData;
        std::vector<depalletizing_mapping::Point3D> mOutputData;

        depalletizing_mapping::Point3D mInitialPoint;
    };
}

#endif //DEPALLETIZING_MAPPING_CORE_QUICKHULL_HPP
