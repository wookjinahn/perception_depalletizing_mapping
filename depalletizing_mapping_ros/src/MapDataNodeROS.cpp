
#include "depalletizing_mapping_ros/MapDataNodeROS.hpp"

namespace depalletizing_mapping
{
	MapDataNodeROS::MapDataNodeROS()
		: depalletizing_mapping::MapDataNode()
	{
        BoundingBox positionRange(0, 0, 1.0f, 1.0f);
        mPositionRange = positionRange;
        std::cout << "positionRange : " << mPositionRange.GetX() << ", " << mPositionRange.GetZ() << ", " << mPositionRange.GetH() << ", " << mPositionRange.GetW() << std::endl;
	}

	MapDataNodeROS::~MapDataNodeROS()
	{
	}

	void MapDataNodeROS::SetRotateDegree(double rotateDegree)
	{
		mRotateDegree = rotateDegree;
	}

	double MapDataNodeROS::GetRotateDegree() const
	{
		return mRotateDegree;
	}

	void MapDataNodeROS::SetRotateRadian(double rotateRadian)
	{
		mRotateDegree = rotateRadian * R2D;
	}

	double MapDataNodeROS::GetRotateRadian() const
	{
		return mRotateDegree * D2R;
	}

    void MapDataNodeROS::SetDataRange(float w, float h)
    {
        BoundingBox dataRange(0, 0, w, h);
        mPositionRange = dataRange;
    }

    void MapDataNodeROS::SetDataRange(BoundingBox dataRange)
    {
        mPositionRange = dataRange;
    }

    BoundingBox MapDataNodeROS::GetDataRange() const
    {
        return mPositionRange;
    }

    void MapDataNodeROS::MakeMapToPointsWithOldData()
    {
        std::cout << "MakeMapToPointsWithOldData" << std::endl;
        std::cout << "GetMapDataPair size : " << GetMapDataPair().size() << std::endl;
        for (auto iter = mOldMapData.begin(); iter != mOldMapData.end(); ++iter)
        {
            GetMapDataPair().insert( {std::make_pair(iter->first.first, iter->first.second), iter->second} );
        }
        std::cout << "GetMapDataPair size : " << GetMapDataPair().size() << std::endl;

        MakeMapToPoints();
//        for (auto iter = GetMapDataPair().begin(); iter != GetMapDataPair().end(); ++iter)
//        {
//            Point3D pointXYZ = { iter->first.first, iter->second, iter->first.second };
//            mOutputPoints.push_back(pointXYZ);
//        }
        std::cout << "MakeMapToPointsWithOldData END" << std::endl;
    }

	void MapDataNodeROS::FromMessage(sensor_msgs::PointCloud pointcloud_msg)
	{
		for(int i = 0; i < pointcloud_msg.points.size(); i++)
		{
			Point3D pointXYZ = {pointcloud_msg.points[i].x, pointcloud_msg.points[i].y, pointcloud_msg.points[i].z };
            SetInputPoint(pointXYZ);
		}
	}

	void MapDataNodeROS::ToMessage(std::string frame_id, sensor_msgs::PointCloud& output_pointcloud)		//camera1_depth_optical_frame
	{
		MakeMapToPoints();

		output_pointcloud.header.frame_id = frame_id;           // header
		output_pointcloud.header.stamp = ros::Time::now();
		output_pointcloud.points.resize(GetOutputPoints().size());     // points			//************************

		for (int i = 0; i < output_pointcloud.points.size(); i++)
		{
			output_pointcloud.points[i].x = GetOutputPoints()[i].GetX();					//************************
			output_pointcloud.points[i].y = GetOutputPoints()[i].GetY();					//************************
			output_pointcloud.points[i].z = GetOutputPoints()[i].GetZ();					//************************
		}

	}



	void MapDataNodeROS::FromMessagePointCloud2(sensor_msgs::PointCloud2 pointcloud2_msgs)
	{
//		MakeMapToPoints();

		sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud2_msgs, "x");
		sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud2_msgs, "y");
		sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud2_msgs, "z");

		for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
			// Check if the point is invalid
			if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
			{
				Point3D pointXYZ = {*iter_x, *iter_y, *iter_z};
                SetInputPoint(pointXYZ);
			}
		}
	}



	void MapDataNodeROS::ToMessagePointCloud2(std::string frame_id, sensor_msgs::PointCloud2& output_pointcloud2)
	{
// ======================= NEED TO !! ===========================
// ======================= NEED TO !! ===========================
// ======================= NEED TO !! ===========================
// ======================= NEED TO !! ===========================
// ======================= NEED TO !! ===========================
	}

	void MapDataNodeROS::ToHeightmapMsgs(depalletizing_mapping_msgs::DepalletizingMap& depalletizing_mapping_msgs, float cameraHeight, Point3D& odom)
	{
        std::cout << "ToHeightmapMsgs" << std::endl;
//		MakeMapToPoints();
        UpdateOldDataByOdom(odom);
        MakeMapToPointsWithOldData();

        std::cout << "ToHeightmapMsgs : to msg" << std::endl;

        depalletizing_mapping_msgs.header.frame_id = "t265_base_frame";           // header
//		depalletizing_mapping_msgs.header.frame_id = "depth_frame";           // header
        depalletizing_mapping_msgs.header.stamp = ros::Time::now();
        depalletizing_mapping_msgs.resolution = 0.03;
        depalletizing_mapping_msgs.points.resize(GetOutputPoints().size());

		float rotationMatrix[3][3] = {{1.0f, 0.0f, 0.0f},
									  {0.0f, (float)std::cos(mRotateDegree * D2R), (float)-std::sin(mRotateDegree * D2R)},
									  {0.0f, (float)std::sin(mRotateDegree * D2R), (float)std::cos(mRotateDegree * D2R)}};

		for (int i = 0; i < this->GetOutputPoints().size(); i++)
		{
			float x = GetOutputPoints()[i].GetX();
			float y = GetOutputPoints()[i].GetY();
			float z = GetOutputPoints()[i].GetZ();
            depalletizing_mapping_msgs.points[i].x = rotationMatrix[0][0] * x + rotationMatrix[0][1] *  y + rotationMatrix[0][2] * z;
            depalletizing_mapping_msgs.points[i].y = -(cameraHeight-(rotationMatrix[1][0] * x + rotationMatrix[1][1] * y + rotationMatrix[1][2] * z));
            depalletizing_mapping_msgs.points[i].z = rotationMatrix[2][0] * x + rotationMatrix[2][1] *  y + rotationMatrix[2][2] * z;

            Point3D pastPoint = {x, y, z};
            mPastData.push_back(pastPoint);
		}
	}

    void MapDataNodeROS::UpdateOldDataByOdom(Point3D& odomPosition)
    {
        std::cout << "UpdateOldDataByOdom" << std::endl;
        float newPositionXRatio = static_cast<int>(odomPosition.GetX() / GetResolution());
        float newPositionYRatio = static_cast<int>(odomPosition.GetY() / GetResolution());

        for (int i = 0; i < mPastData.size(); i++)
        {
            Point3D oldPoint(mPastData[i].GetX() + (-newPositionYRatio) * GetResolution(), mPastData[i].GetY(), mPastData[i].GetZ() + newPositionXRatio * GetResolution());

            if (mPositionRange.IsConstained(oldPoint))
            {
                mOldMapData.insert({ std::make_pair(oldPoint.GetNodeKey().GetX(), oldPoint.GetNodeKey().GetZ()), oldPoint.GetY() });
            }
        }
        std::cout << "mOldMapData : " << mOldMapData.size() << std::endl;
    }
}