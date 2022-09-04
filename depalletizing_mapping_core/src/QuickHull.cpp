#include "depalletizing_mapping_core/QuickHull.hpp"

namespace depalletizing_mapping
{
    QuickHull::QuickHull()
    {
    }

    QuickHull::QuickHull(std::vector<depalletizing_mapping::Point3D>& inputPoint)
    {
        mInputData = inputPoint;
    }

    void QuickHull::SetInputData(std::vector<depalletizing_mapping::Point3D>& inputPoint)
    {
        mInputData = inputPoint;
    }

    std::vector<depalletizing_mapping::Point3D> QuickHull::GetInputData() const
    {
        return mInputData;
    }

    void QuickHull::SetOutputData(std::vector<depalletizing_mapping::Point3D>& outPoint)
    {
        mOutputData = outPoint;
    }

    std::vector<depalletizing_mapping::Point3D> QuickHull::GetOutputData() const
    {
        return mOutputData;
    }

    void QuickHull::Run()
    {
        if (mInputData.size() < 3)
        {
            std::cout << "Convex hull not possible" << std::endl;
            return;
        }

        int min_x = 0, max_x = 0;
        for (int i=1; i<mInputData.size(); i++)
        {
            if (mInputData[i].GetX() < mInputData[min_x].GetX())
                min_x = i;
            if (mInputData[i].GetX() > mInputData[max_x].GetX())
                max_x = i;
        }

        quickHull(mInputData, mInputData[min_x], mInputData[max_x], 1);
        quickHull(mInputData, mInputData[min_x], mInputData[max_x], -1);
    }

    int QuickHull::findSide(depalletizing_mapping::Point3D point1, depalletizing_mapping::Point3D point2, depalletizing_mapping::Point3D point)
    {
        float val = (point.GetZ() - point1.GetZ()) * (point2.GetX() - point1.GetX()) -
                    (point2.GetZ() - point1.GetZ()) * (point.GetX() - point1.GetX());

        if (val > 0)
            return 1;
        if (val < 0)
            return -1;
        return 0;
    }

    float QuickHull::lineDist(depalletizing_mapping::Point3D point1, depalletizing_mapping::Point3D point2, depalletizing_mapping::Point3D point)
    {
        return std::abs((point.GetZ() - point1.GetZ()) * (point2.GetX() - point1.GetX()) -
                        (point2.GetZ() - point1.GetZ()) * (point.GetX() - point1.GetX()));
    }

    void QuickHull::quickHull(const std::vector<depalletizing_mapping::Point3D>& data, depalletizing_mapping::Point3D point1, depalletizing_mapping::Point3D point2, int side)
    {
        int ind = -1;
        float max_dist = 0.0f;

        for (int i=0; i<data.size(); i++)
        {
            float temp = lineDist(point1, point2, data[i]);
            if (findSide(point1, point2, data[i]) == side && temp > max_dist)
            {
                ind = i;
                max_dist = temp;
            }
        }

        if (ind == -1)
        {
            mOutputData.push_back(point1);
            mOutputData.push_back(point2);
            return;
        }

        quickHull(data, data[ind], point1, -findSide(data[ind], point1, point2));
        quickHull(data, data[ind], point2, -findSide(data[ind], point2, point1));

        getAngleEachPoints();
        sortByAngle();
    }

    void QuickHull::getAngleEachPoints()
    {
        std::sort(mOutputData.begin(), mOutputData.end(), depalletizing_mapping::Point3D::AscendingByZ);

        mInitialPoint = mOutputData[0];
        for (int i = 0; i < mOutputData.size(); i++)
        {
            if (mOutputData[i].GetX() == mInitialPoint.GetX() && mOutputData[i].GetY() == mInitialPoint.GetY() && mOutputData[i].GetZ() == mInitialPoint.GetZ())
            {
                mOutputData.erase(mOutputData.begin() + i);
                i++;
            }
        }
        for (int i = 0; i < mOutputData.size(); i++)
        {
            if (mOutputData[i].GetX() == mInitialPoint.GetX() && mOutputData[i].GetY() == mInitialPoint.GetY() && mOutputData[i].GetZ() == mInitialPoint.GetZ())
            {
                mOutputData.erase(mOutputData.begin() + i);
                i++;
            }
        }

        std::vector<float> angles;
        for (int i = 0; i < mOutputData.size(); i++)
        {
            float angle = std::atan2(mOutputData[i].GetX() - mInitialPoint.GetX(), mOutputData[i].GetZ() - mInitialPoint.GetZ());
            float distance = mOutputData[i].DistanceBetweenOther(mInitialPoint);
            mOutputData[i].SetAngle(angle);
            mOutputData[i].SetDistance(distance);
        }
    }

    bool QuickHull::DescendingByAngle(depalletizing_mapping::Point3D& firstPoint, depalletizing_mapping::Point3D& secondPoint)
    {
        if (firstPoint.GetAngle() == secondPoint.GetAngle())
        {
            return firstPoint.GetDistance() > secondPoint.GetDistance();
        }
        else
        {
            return firstPoint.GetAngle() > secondPoint.GetAngle();
        }
    }

    void QuickHull::sortByAngle()
    {
        std::sort(mOutputData.begin(), mOutputData.end(), DescendingByAngle);

        std::vector<depalletizing_mapping::Point3D> output;
        output.push_back(mInitialPoint);

        for (int i = 0; i < mOutputData.size(); i++)
        {
            output.push_back(mOutputData[i]);
        }
        mOutputData.clear();
        mOutputData = output;
    }
}

