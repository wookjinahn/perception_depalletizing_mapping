
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

    void MapDataNodeROS::SetCameraHeight(float cameraHeight)
    {
        mCameraHeight = cameraHeight;
    }

    float MapDataNodeROS::GetCameraHeight() const
    {
        return mCameraHeight;
    }

    void MapDataNodeROS::FromPointCloud2Msgs(sensor_msgs::PointCloud2 pointcloud2_msgs)
    {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud2_msgs, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud2_msgs, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud2_msgs, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
            if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
            {
                Point3D pointXYZ = { *iter_x, *iter_z, *iter_y };
//                Point3D pointXYZ = { *iter_x, *iter_y, *iter_z };
                SetInputPoint(pointXYZ);
            }
        }
    }

    std::vector<float> MapDataNodeROS::ToPointCloud2Msgs(std::string frame_id, sensor_msgs::PointCloud2& output_pointcloud2)
    {
        std::vector<Point3D> detectedPlanarPoints = GetDetectedPlanarPoints();
        std::vector<float> centerPoint;
        centerPoint.reserve(3);
        float planarCenterX = 0;
        float planarCenterY = 0;
        float planarCenterZ = 0;

        sensor_msgs::PointCloud pointcloud_msgs;

        pointcloud_msgs.header.frame_id = frame_id;
        pointcloud_msgs.header.stamp = ros::Time::now();
        pointcloud_msgs.points.resize(detectedPlanarPoints.size());

        for (int i = 0; i < pointcloud_msgs.points.size(); i++) {
            Point3D outData = detectedPlanarPoints[i];
            pointcloud_msgs.points[i].x = outData.GetX();
            pointcloud_msgs.points[i].y = outData.GetY();
            pointcloud_msgs.points[i].z = -(outData.GetZ());
            planarCenterX += -(outData.GetZ());
            planarCenterY += -(outData.GetX());
            planarCenterZ += -(outData.GetY()) + 0.016;
        }

        sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msgs, output_pointcloud2);

        centerPoint = { planarCenterX / detectedPlanarPoints.size(), planarCenterY / detectedPlanarPoints.size(), planarCenterZ / detectedPlanarPoints.size() };
        return centerPoint;
    }


    void MapDataNodeROS::ToDepalletizingMapMsgs(std::string frame_id, depalletizing_mapping_msgs::DepalletizingMap& depalletizing_mapping_msgs)
    {
        depalletizing_mapping_msgs.header.frame_id = frame_id;
        depalletizing_mapping_msgs.header.stamp = ros::Time::now();
        depalletizing_mapping_msgs.resolution = GetResolution();
        depalletizing_mapping_msgs.points.resize(GetOutputPoints().size());

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

    void MapDataNodeROS::ToPolygonMsgs(std::string frame_id, geometry_msgs::PolygonStamped& polygon_msgs)
    {
        polygon_msgs.header.frame_id = frame_id;
        polygon_msgs.header.stamp = ros::Time::now();
        polygon_msgs.polygon.points.resize(GetDetectedPlanarPolygon().size());

        for (int i = 0; i < GetDetectedPlanarPolygon().size(); i++)
        {
            float x = GetDetectedPlanarPolygon()[i].GetX();
            float y = GetDetectedPlanarPolygon()[i].GetY();
            float z = -(GetDetectedPlanarPolygon()[i].GetZ());

            polygon_msgs.polygon.points[i].x = x;
            polygon_msgs.polygon.points[i].y = y;
            polygon_msgs.polygon.points[i].z = z;
        }
    }
}