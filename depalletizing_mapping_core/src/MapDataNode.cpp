
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
        runOutlierRemoval();
        runQuickHull();
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

        std::vector<Point3D> detectedPlanarPoints = mKMeans.GetOutputData();
        SetDetectedPlanarPoints(detectedPlanarPoints);
    }

    void MapDataNode::runQuickHull()
    {
        mQuickHull.SetInputData(mDetectedPlanarPoints);
        mQuickHull.Run();

        std::vector<Point3D> detectedPlanarPolygon = mQuickHull.GetOutputData();
        SetDetectedPlanarPolygon(detectedPlanarPolygon);

    }

    void MapDataNode::runOutlierRemoval()
    {
        std::sort(mDetectedPlanarPoints.begin(), mDetectedPlanarPoints.end(), Point3D::DescendingByX);
        float maxX = mDetectedPlanarPoints[0].GetX();
        float minX = mDetectedPlanarPoints[mDetectedPlanarPoints.size() - 1].GetX();
        float xCenter = (maxX + minX) / 2;

        std::sort(mDetectedPlanarPoints.begin(), mDetectedPlanarPoints.end(), Point3D::DescendingByZ);
        float maxZ = mDetectedPlanarPoints[0].GetZ();
        float minZ = mDetectedPlanarPoints[mDetectedPlanarPoints.size() - 1].GetZ();
        float zCenter = (maxZ + minZ) / 2;

        int filterGridXCount = abs(static_cast<int>((maxX - xCenter) / mResolution));
        int filterGridZCount = abs(static_cast<int>((maxZ - zCenter) / mResolution));

        float dataRangeWidth = static_cast<float>(filterGridXCount) * static_cast<float>(mResolution);
        float dataRangeHeight = static_cast<float>(filterGridZCount) * static_cast<float>(mResolution);

        BoundingBox filteredBoundary(xCenter, zCenter, dataRangeWidth, dataRangeHeight);

        mFilteredDataMat = new float* [filterGridXCount * 2];
        mVisitedMat = new int* [filterGridXCount * 2];
        for (int i = 0; i < filterGridXCount * 2; i++)
        {
            mFilteredDataMat[i] = new float[filterGridZCount * 2];
            mVisitedMat[i] = new int[filterGridZCount * 2];
        }

        for (int i = 0; i < filterGridXCount * 2; i++)
        {
            for (int j = 0; j < filterGridZCount * 2; j++)
            {
                mFilteredDataMat[i][j] = -10.0f;
                mVisitedMat[i][j] = 0;
            }
        }

        int rawDataNum = 0;
        for (auto& points : mDetectedPlanarPoints)
        {
            if (filteredBoundary.IsConstained(points))
            {
                int xIndex = static_cast<int>(points.GetX() / mResolution) + filterGridXCount - static_cast<int>(xCenter / mResolution);
                int zIndex = static_cast<int>(points.GetZ() / mResolution) + filterGridZCount - static_cast<int>(zCenter / mResolution);

                if (xIndex >= 0 && xIndex < filterGridXCount * 2 && zIndex >= 0 && zIndex < filterGridZCount * 2)
                {
                    rawDataNum++;
                    mFilteredDataMat[xIndex][zIndex] = points.GetY();
                }
            }
        }

        std::vector<Point3D> filteredData;
        filteredData.reserve(mDetectedPlanarPoints.size());

        for (int i = 0; i < filterGridXCount * 2; i++)
        {
            for (int j = 0; j < filterGridZCount * 2; j++)
            {
                filteredData.clear();

                if (mFilteredDataMat[i][j] != -10.0f && mVisitedMat[i][j] == 0)
                {
                    filteringData(i, j, mVisitedMat, filteredData, filterGridXCount * 2, filterGridZCount * 2);
                }

                if (static_cast<float>(filteredData.size()) / rawDataNum > 0.5f)
                {
                    mDetectedPlanarPoints.clear();
                    for (int i = 0; i < filteredData.size(); i++)
                    {
                        float x = static_cast<float>(filteredData[i].GetX() + static_cast<int>(xCenter / mResolution) - filterGridXCount) * mResolution;
                        float y = filteredData[i].GetY();
                        float z = static_cast<float>(filteredData[i].GetZ() + static_cast<int>(zCenter / mResolution) - filterGridZCount) * mResolution;
                        Point3D point(x, y, z);
                        mDetectedPlanarPoints.push_back(point);
                    }

                    for (int i = 0; i < filterGridXCount * 2; i++)
                    {
                        delete[] mFilteredDataMat[i];
                    }
                    delete[] mFilteredDataMat;

                    return;
                }
            }
        }
    }

    void MapDataNode::filteringData(int xIndex, int zIndex, int** visited, std::vector<Point3D>& filteredData, int gridX, int gridZ)
    {
        visited[xIndex][zIndex] = 1;
        filteredData.push_back(Point3D(static_cast<float>(xIndex), mFilteredDataMat[xIndex][zIndex], static_cast<float>(zIndex)));

        int dx[4] = {1, 0, -1, 0};
        int dz[4] = {0, 1, 0, -1};

        for (int i = 0; i < 4; i++)
        {
            int nextX = xIndex + dx[i];
            int nextZ = zIndex + dz[i];

            if (nextX >= 0 && nextX < gridX && nextZ >= 0 && nextZ < gridZ)
            {
                if (mFilteredDataMat[nextX][nextZ] != -10.0f && mVisitedMat[nextX][nextZ] == 0)
                filteringData(nextX, nextZ, visited, filteredData, gridX, gridZ);
            }
        }
    }
}