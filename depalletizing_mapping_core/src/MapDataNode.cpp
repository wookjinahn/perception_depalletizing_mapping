
#include "depalletizing_mapping_core/MapDataNode.hpp"

namespace depalletizing_mapping
{
//    DepalletizingMapping::DepalletizingMapping(Ransac& ransac, Model::Plane& planeModle)
//		: mRansac(ransac)
//		, mPlaneModel(planeModle)
//    {
//    }
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

    KMeans MapDataNode::GetKMeans() const
    {
        return mKMeans;
    }

    void MapDataNode::SetKMeans(KMeans& kmeans)
    {
        mKMeans = kmeans;
    }

    std::vector<PlaneModel> MapDataNode::GetDetectedPlanes() const
    {
        return mDetectedPlanes;
    }

    void MapDataNode::SetDetectedPlanes(std::vector<PlaneModel>& detectedPlanes)
    {
        mDetectedPlanes = detectedPlanes;
    }

    void MapDataNode::RunRansac()
    {
        mRansac.SetMaxIteration(GetInputPoints().size());

        std::vector<Point3D> data = GetOutputPoints();
        std::sort(data.begin(), data.end(), Point3D::AscendingByY);
        mRansac.SetData(data);
        mRansac.RunMulti();
    }

    void MapDataNode::RunKMeansClustering()
    {
        mKMeans.SetData(mRansac.GetResultModel()[0].GetData());
        mKMeans.Run();
        mKMeans.SaveOneResult();
    }
}