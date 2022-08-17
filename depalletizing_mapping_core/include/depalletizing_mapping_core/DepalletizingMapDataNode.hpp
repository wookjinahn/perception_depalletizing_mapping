#ifndef DEPALLETIZING_MAPPING_CORE_DEPALLETIZINGMAPDATANODE_HPP
#define DEPALLETIZING_MAPPING_CORE_DEPALLETIZINGMAPDATANODE_HPP

#include <depalletizing_mapping_base/depalletizing_mapping_base.hpp>

#include "KMeans.hpp"
#include "Ransac.hpp"
#include "Plane.hpp"

namespace depalletizing_mapping
{
    class DepalletizingMapping : public depalletizing_mapping::MapDataNode
    {
    public:
        DepalletizingMapping(Ransac& ransac);
        ~DepalletizingMapping();

        Model::Plane GetPlaneModel() const;
        void SetPlaneModel(Model::Plane& planeModel);
        Ransac GetRansac() const;
        void SetRansac(Ransac& ransac);
        KMeans GetKMeans() const;
        void SetKMeans(KMeans& kmeans);
        std::vector<Model::Plane> GetDetectedPlanes() const;
        void SetDetectedPlanes(std::vector<Model::Plane>& detectedPlanes);

        void ToPCDForDepalletizer(const std::string& outputPath);
        void FromMessagePointCloud2ForDepalletizer(sensor_msgs::PointCloud2 pointcloud2_msgs);
        void FromMessagePointCloudForDepalletizer(sensor_msgs::PointCloud pointcloud_msgs);
        void ToMessageForDepalletizer(std::string frame_id, sensor_msgs::PointCloud& output_pointcloud);
        std::vector<float> ToMessageForDepalletizerWithCenter(std::string frame_id, sensor_msgs::PointCloud& output_pointcloud);
        void ToHeightmapMsgsForDepalletizer(std::string frame_id, heightmap_msgs::Heightmap& heightmap_msgs, float cameraHeight);

    private:
        void runRansac();
        void runKMeansClustering();

        KMeans mKMeans;
        Ransac mRansac;
        Model::Plane mPlaneModel;
        std::vector<Model::Plane> mDetectedPlanes;
    };
}

#endif //DEPALLETIZING_MAPPING_CORE_DEPALLETIZINGMAPDATANODE_HPP
