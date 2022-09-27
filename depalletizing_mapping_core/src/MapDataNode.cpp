
#include "depalletizing_mapping_core/MapDataNode.hpp"

namespace depalletizing_mapping
{
    MapDataNode::MapDataNode()
    {
    }

    MapDataNode::MapDataNode(Ransac& ransac)
            : mRansac(ransac)
    {
    }

    MapDataNode::~MapDataNode()
    {
    }

    PlaneModel MapDataNode::GetPlaneModel() const
    {
        return mPlaneModel;
    }

    void MapDataNode::SetPlaneModel(PlaneModel& planeModel)
    {
        mPlaneModel = planeModel;
    }

    Ransac MapDataNode::GetRansac() const
    {
        return mRansac;
    }

    void MapDataNode::SetRansac(Ransac& ransac)
    {
        mRansac = ransac;
    }

    void MapDataNode::SetRansacParams(float threshold, int upperPointsNum)
    {
        mRansac.SetParams(threshold, upperPointsNum);
    }

    KMeans MapDataNode::GetKMeans() const
    {
        return mKMeans;
    }

    void MapDataNode::SetKMeans(KMeans& kmeans)
    {
        mKMeans = kmeans;
    }

    QuickHull MapDataNode::GetQuickHull() const
    {
        return mQuickHull;
    }

    void MapDataNode::SetQuickHull(QuickHull& quickHull)
    {
        mQuickHull = quickHull;
    }

    std::vector<Point3D> MapDataNode::GetDetectedPlanarPoints() const
    {
        return mDetectedPlanarPoints;
    }

    void MapDataNode::SetDetectedPlanarPoints(const std::vector<Point3D>& detectedPlanarPoints)
    {
        mDetectedPlanarPoints = detectedPlanarPoints;
    }

    std::vector<Point3D> MapDataNode::GetDetectedPlanarPolygon() const
    {
        return mDetectedPlanarPolygon;
    }

    void MapDataNode::SetDetectedPlanarPolygon(const std::vector<Point3D>& detectedPlanarPolygon)
    {
        mDetectedPlanarPolygon =detectedPlanarPolygon;
    }

    void MapDataNode::DetectPlanarRegion()
    {
        runRansac();
        runKMeansClustering();
        runQuickHull();

        std::vector<Point3D> detectedPlanarPoints = mKMeans.GetOutputData();
        SetDetectedPlanarPoints(detectedPlanarPoints);

        std::vector<Point3D> detectedPlanarPolygon = mQuickHull.GetOutputData();
        SetDetectedPlanarPolygon(detectedPlanarPolygon);
    }

    void MapDataNode::runRansac()
    {
        if (mRansac.GetMaxIteration() != 0)
        {
            mRansac.SetMaxIteration(GetInputPoints().size());
        }

        std::vector<Point3D> data = GetOutputPoints();
        std::sort(data.begin(), data.end(), Point3D::AscendingByY);
        mRansac.SetData(data);
        mRansac.RunMulti();
    }

    void MapDataNode::runKMeansClustering()
    {
        mKMeans.SetData(mRansac.GetResultModel()[0].GetData());
        mKMeans.Run();
        mKMeans.SaveOneResult();
    }

    void MapDataNode::runQuickHull()
    {
        std::vector<Point3D> detectedPlanarPoints = mKMeans.GetOutputData();
        mQuickHull.SetInputData(detectedPlanarPoints);
        mQuickHull.Run();
    }
}