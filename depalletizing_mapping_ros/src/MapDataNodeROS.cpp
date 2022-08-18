
#include "depalletizing_mapping_ros/MapDataNodeROS.hpp"

namespace depalletizing_mapping
{
	MapDataNodeROS::MapDataNodeROS()
		: depalletizing_mapping::MapDataNode()
	{
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

    void MapDataNodeROS::ToPCDForDepalletizer(const std::string& outputPath)
    {
        MakeMapToPoints();

        time_t t;
        struct tm* timeinfo;
        time(&t);
        timeinfo = localtime(&t);

        std::string hour, min;

        if (timeinfo->tm_hour < 10) hour = "0" + std::to_string(timeinfo->tm_hour);
        else hour = std::to_string(timeinfo->tm_hour);

        if (timeinfo->tm_min < 10) min = "0" + std::to_string(timeinfo->tm_min);
        else min = std::to_string(timeinfo->tm_min);

        std::string filePath = outputPath + hour + min + ".pcd";

        std::ofstream fout;
        fout.open(filePath);

        fout << "VERSION" << std::endl;
        fout << "FIELDS x y z" << std::endl;
        fout << "SIZE 4 4 4" << std::endl;
        fout << "TYPE F F F" << std::endl;
        fout << "COUNT 1 1 1" << std::endl;
        fout << "WIDTH 1" << std::endl;
        fout << "HEIGHT " << GetOutputPoints().size() << std::endl;
        fout << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
        fout << "POINTS " << GetOutputPoints().size() << std::endl;
        fout << "DATA ascii" << std::endl;

        for (int i = 0; i < GetOutputPoints().size(); i++)
        {
            fout << GetOutputPoints()[i].GetX() << " " << GetOutputPoints()[i].GetY() << " " << -(GetOutputPoints()[i].GetZ()) << "\n";
        }

        fout.close();
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
        RunRansac();
        RunKMeansClustering();

//        output_pointcloud.header.frame_id = frame_id;           // header
//        output_pointcloud.header.stamp = ros::Time::now();
//        output_pointcloud.points.resize(GetRansac().GetResultModel()[0].GetData().size());     // points			//************************
//
//        for (int i = 0; i < output_pointcloud.points.size(); i++) {
//            output_pointcloud.points[i].x = GetRansac().GetResultModel()[0].GetData()[i].GetX();                    //************************
//            output_pointcloud.points[i].y = GetRansac().GetResultModel()[0].GetData()[i].GetY();                    //************************
//            output_pointcloud.points[i].z = -(GetRansac().GetResultModel()[0].GetData()[i].GetZ());                    //************************
//        }

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

        RunRansac();
        RunKMeansClustering();

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


    void MapDataNodeROS::ToHeightmapMsgsForDepalletizer(std::string frame_id, depalletizing_mapping_msgs::DepalletizingMap& depalletizing_mapping_msgs, float cameraHeight)
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
            depalletizing_mapping_msgs.points[i].y = -(cameraHeight-y);
            depalletizing_mapping_msgs.points[i].z = z;

        }
    }
}