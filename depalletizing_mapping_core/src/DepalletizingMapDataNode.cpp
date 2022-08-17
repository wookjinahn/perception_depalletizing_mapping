
#include "depalletizing_mapping_core/DepalletizingDataNode.hpp"

namespace camel
{
//    DepalletizingMapping::DepalletizingMapping(Ransac& ransac, Model::Plane& planeModle)
//		: mRansac(ransac)
//		, mPlaneModel(planeModle)
//    {
//    }

    DepalletizingMapping::DepalletizingMapping(Ransac& ransac)
            : mRansac(ransac)
    {
    }

    DepalletizingMapping::~DepalletizingMapping()
    {
    }

    Model::Plane DepalletizingMapping::GetPlaneModel() const
    {
        return mPlaneModel;
    }

    void DepalletizingMapping::SetPlaneModel(Model::Plane& planeModel)
    {
        mPlaneModel = planeModel;
    }

    Ransac DepalletizingMapping::GetRansac() const
    {
        return mRansac;
    }

    void DepalletizingMapping::SetRansac(Ransac& ransac)
    {
        mRansac = ransac;
    }

    KMeans DepalletizingMapping::GetKMeans() const
    {
        return mKMeans;
    }

    void DepalletizingMapping::SetKMeans(KMeans& kmeans)
    {
        mKMeans = kmeans;
    }

    std::vector<Model::Plane> DepalletizingMapping::GetDetectedPlanes() const
    {
        return mDetectedPlanes;
    }

    void DepalletizingMapping::SetDetectedPlanes(std::vector<Model::Plane>& detectedPlanes)
    {
        mDetectedPlanes = detectedPlanes;
    }

    void DepalletizingMapping::ToPCDForDepalletizer(const std::string& outputPath)
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

    void DepalletizingMapping::FromMessagePointCloud2ForDepalletizer(sensor_msgs::PointCloud2 pointcloud2_msgs)
    {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud2_msgs, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud2_msgs, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud2_msgs, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
            // Check if the point is invalid
            if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
            {
                Point3 pointXYZ = { *iter_x, *iter_z, *iter_y };
                SetInputPoint(pointXYZ);
            }
        }
    }

    void DepalletizingMapping::FromMessagePointCloudForDepalletizer(sensor_msgs::PointCloud pointcloud_msgs)
    {
        for (int i = 0; i < pointcloud_msgs.points.size(); i++)
        {
            Point3 pointXYZ = { pointcloud_msgs.points[i].x, pointcloud_msgs.points[i].z, pointcloud_msgs.points[i].y };
            SetInputPoint(pointXYZ);
        }

    }

    void DepalletizingMapping::ToMessageForDepalletizer(std::string frame_id, sensor_msgs::PointCloud& output_pointcloud)		//camera1_depth_optical_frame
    {
        runRansac();
        runKMeansClustering();

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

    std::vector<float> DepalletizingMapping::ToMessageForDepalletizerWithCenter(std::string frame_id, sensor_msgs::PointCloud& output_pointcloud)		//camera1_depth_optical_frame
    {
        std::vector<float> centerPoint;
        centerPoint.reserve(3);
        float planarCenterX = 0;
        float planarCenterY = 0;
        float planarCenterZ = 0;

        runRansac();
        runKMeansClustering();

        output_pointcloud.header.frame_id = frame_id;           // header
        output_pointcloud.header.stamp = ros::Time::now();
        output_pointcloud.points.resize(GetKMeans().GetOutputData().size());     // points			//************************

        for (int i = 0; i < output_pointcloud.points.size(); i++) {
            camel::Point3 outData = GetKMeans().GetOutputData()[i];
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


    void DepalletizingMapping::ToHeightmapMsgsForDepalletizer(std::string frame_id, heightmap_msgs::Heightmap& heightmap_msgs, float cameraHeight)
    {
        heightmap_msgs.header.frame_id = frame_id;           // header
        heightmap_msgs.header.stamp = ros::Time::now();
        heightmap_msgs.resolution = GetResolution();
        heightmap_msgs.points.resize(GetOutputPoints().size());

        // outputpoints -> (x, z, y)
        for (int i = 0; i < GetOutputPoints().size(); i++)
        {
            float x = GetOutputPoints()[i].GetX();
            float y = GetOutputPoints()[i].GetY();
            float z = -(GetOutputPoints()[i].GetZ());
            heightmap_msgs.points[i].x = x;
            heightmap_msgs.points[i].y = -(cameraHeight-y);
            heightmap_msgs.points[i].z = z;

        }
    }

    void DepalletizingMapping::runRansac()
    {
        // sort
        // -> set ransac parameters
//        mRansac.SetData(this->GetOutputPoints())
//
        mRansac.SetMaxIteration(GetInputPoints().size());

        std::vector<camel::Point3> data;
//        for (int i = 0; i < GetOutputPoints().size(); i++)
//        {
//            camel::Point3 inputData = {GetOutputPoints()[i].GetX(), GetOutputPoints()[i].GetY(), GetOutputPoints()[i].GetZ() };
//            data.push_back(inputData);
//        }
        data = GetOutputPoints();
        std::sort(data.begin(), data.end(), camelVector::Point3D::AscendingByY);
        mRansac.SetData(data);
        mRansac.RunMulti();
    }

    void DepalletizingMapping::runKMeansClustering()
    {
        mKMeans.SetData(mRansac.GetResultModel()[0].GetData());
        mKMeans.Run();
        mKMeans.SaveOneResult();
    }
}