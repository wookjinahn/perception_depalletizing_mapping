#ifndef DEPALLETIZING_MAPPING_CORE_MAPDATANODE_HPP
#define DEPALLETIZING_MAPPING_CORE_MAPDATANODE_HPP

#include <depalletizing_mapping_base/depalletizing_mapping_base.hpp>

#include "KMeans.hpp"
#include "PlaneModel.hpp"
#include "QuickHull.hpp"
#include "Ransac.hpp"

namespace depalletizing_mapping
{
    class MapDataNode : public depalletizing_mapping::DataNode
    {
    public:
        MapDataNode();
        MapDataNode(Ransac& ransac);
        ~MapDataNode();

        PlaneModel GetPlaneModel() const;
        void SetPlaneModel(PlaneModel& planeModel);
        Ransac GetRansac() const;
        void SetRansac(Ransac& ransac);
        void SetRansacParams(float threshold, int upperPointsNum);
        KMeans GetKMeans() const;
        void SetKMeans(KMeans& kmeans);
        QuickHull GetQuickHull() const;
        void SetQuickHull(QuickHull& quickHull);
        std::vector<Point3D> GetDetectedPlanarPoints() const;
        void SetDetectedPlanarPoints(const std::vector<Point3D>& detectedPlanarPoints);
        std::vector<Point3D> GetDetectedPlanarPolygon() const;
        void SetDetectedPlanarPolygon(const std::vector<Point3D>& detectedPlanarPolygon);

        void DetectPlanarRegion();

    private:
        void runRansac();
        void runKMeansClustering();
        void runQuickHull();
        void runOutlierRemoval();
        void filteringData(int xIndex, int zIndex, int** visited, std::vector<Point3D>& filteredData, int gridX, int gridZ);

        Ransac mRansac;
        PlaneModel mPlaneModel;
        KMeans mKMeans;
        QuickHull mQuickHull;

        std::vector<Point3D> mDetectedPlanarPoints;
        std::vector<Point3D> mDetectedPlanarPolygon;

        float** mFilteredDataMat;
        int** mVisitedMat;
        std::vector<Point3D> mFilteredData;
    };
}

#endif //DEPALLETIZING_MAPPING_CORE_MAPDATANODE_HPP
