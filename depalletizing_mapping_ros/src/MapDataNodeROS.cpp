
#include "depalletizing_mapping_ros/MapDataNodeROS.hpp"

namespace depalletizing_mapping
{
	MapDataNodeROS::MapDataNodeROS()
		: depalletizing_mapping::MapDataNode()
	{
        Ransac ransac;
        SetRansac(ransac);
	}

    MapDataNodeROS::MapDataNodeROS(Ransac& ransac)
        : depalletizing_mapping::MapDataNode(ransac)
    {
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

    void MapDataNodeROS::SetCameraHeight(float cameraHeight)
    {
        mCameraHeight = cameraHeight;
    }

    float MapDataNodeROS::GetCameraHeight() const
    {
        return mCameraHeight;
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
	}

    void MapDataNodeROS::FromMessagePointCloud2ForDepalletizer(sensor_msgs::PointCloud2 pointcloud2_msgs)
    {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud2_msgs, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud2_msgs, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud2_msgs, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
            // Check if the point is invalid
            if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
            {
                Point3D pointXYZ = { *iter_x, *iter_z, *iter_y };
                SetInputPoint(pointXYZ);
            }
        }
    }

    void MapDataNodeROS::FromMessagePointCloudForDepalletizer(sensor_msgs::PointCloud pointcloud_msgs)
    {
        for (int i = 0; i < pointcloud_msgs.points.size(); i++)
        {
            Point3D pointXYZ = { pointcloud_msgs.points[i].x, pointcloud_msgs.points[i].z, pointcloud_msgs.points[i].y };
            SetInputPoint(pointXYZ);
        }

    }

    void MapDataNodeROS::ToMessageForDepalletizer(std::string frame_id, sensor_msgs::PointCloud& output_pointcloud)		//camera1_depth_optical_frame
    {
        Run();

        output_pointcloud.header.frame_id = frame_id;           // header
        output_pointcloud.header.stamp = ros::Time::now();
        output_pointcloud.points.resize(GetKMeans().GetOutputData().size());     // points			//************************

        for (int i = 0; i < output_pointcloud.points.size(); i++) {
            output_pointcloud.points[i].x = GetKMeans().GetOutputData()[i].GetX();                    //************************
            output_pointcloud.points[i].y = GetKMeans().GetOutputData()[i].GetY();                    //************************
            output_pointcloud.points[i].z = -(GetKMeans().GetOutputData()[i].GetZ());                    //************************
        }
    }

    std::vector<float> MapDataNodeROS::ToMessageForDepalletizerWithCenter(std::string frame_id, sensor_msgs::PointCloud& output_pointcloud)		//camera1_depth_optical_frame
    {
        std::vector<float> centerPoint;
        centerPoint.reserve(3);
        float planarCenterX = 0;
        float planarCenterY = 0;
        float planarCenterZ = 0;

        Run();

        output_pointcloud.header.frame_id = frame_id;           // header
        output_pointcloud.header.stamp = ros::Time::now();
        output_pointcloud.points.resize(GetKMeans().GetOutputData().size());     // points			//************************

        for (int i = 0; i < output_pointcloud.points.size(); i++) {
            Point3D outData = GetKMeans().GetOutputData()[i];
            output_pointcloud.points[i].x = outData.GetX();                    //************************
            output_pointcloud.points[i].y = outData.GetY();                    //************************
            output_pointcloud.points[i].z = -(outData.GetZ());                    //************************
            planarCenterX += -(outData.GetZ());
            planarCenterY += -(outData.GetX());
            planarCenterZ += -(outData.GetY()) + 0.016;
        }

        centerPoint = { planarCenterX / GetKMeans().GetOutputData().size(), planarCenterY / GetKMeans().GetOutputData().size(), planarCenterZ / GetKMeans().GetOutputData().size() };
        return centerPoint;
    }

    std::vector<float> MapDataNodeROS::ToMessageForDepalletizerWithCenter(std::string frame_id, sensor_msgs::PointCloud2& output_pointcloud)		//camera1_depth_optical_frame
    {
        sensor_msgs::PointCloud pointcloud_msgs;

        std::vector<float> centerPoint;
        centerPoint.reserve(3);
        float planarCenterX = 0;
        float planarCenterY = 0;
        float planarCenterZ = 0;

        Run();
//
//        output_pointcloud.header.frame_id = frame_id;           // header
//        output_pointcloud.header.stamp = ros::Time::now();
//        output_pointcloud.points.resize(GetKMeans().GetOutputData().size());     // points			//************************
        int outputPointSize = GetKMeans().GetOutputData().size();

        //************************
        pointcloud_msgs.header.frame_id = frame_id;           // header
        pointcloud_msgs.header.stamp = ros::Time::now();
        pointcloud_msgs.points.resize(outputPointSize);     // points			//************************

        for (int i = 0; i < pointcloud_msgs.points.size(); i++) {
            Point3D outData = GetKMeans().GetOutputData()[i];
            pointcloud_msgs.points[i].x = outData.GetX();                    //************************
            pointcloud_msgs.points[i].y = outData.GetY();                    //************************
            pointcloud_msgs.points[i].z = -(outData.GetZ());                    //************************
            planarCenterX += -(outData.GetZ());
            planarCenterY += -(outData.GetX());
            planarCenterZ += -(outData.GetY()) + 0.016;
        }

        sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msgs, output_pointcloud);

        centerPoint = { planarCenterX / GetKMeans().GetOutputData().size(), planarCenterY / GetKMeans().GetOutputData().size(), planarCenterZ / GetKMeans().GetOutputData().size() };
        return centerPoint;
    }


    void MapDataNodeROS::ToHeightmapMsgsForDepalletizer(std::string frame_id, depalletizing_mapping_msgs::DepalletizingMap& depalletizing_mapping_msgs)
    {
        depalletizing_mapping_msgs.header.frame_id = frame_id;           // header
        depalletizing_mapping_msgs.header.stamp = ros::Time::now();
        depalletizing_mapping_msgs.resolution = GetResolution();
        depalletizing_mapping_msgs.points.resize(GetOutputPoints().size());

        // outputpoints -> (x, z, y)
        for (int i = 0; i < GetOutputPoints().size(); i++)
        {
            float x = GetOutputPoints()[i].GetX();
            float y = GetOutputPoints()[i].GetY();
            float z = -(GetOutputPoints()[i].GetZ());
            depalletizing_mapping_msgs.points[i].x = x;
            depalletizing_mapping_msgs.points[i].y = -(mCameraHeight-y);
            depalletizing_mapping_msgs.points[i].z = z;
        }
    }
}