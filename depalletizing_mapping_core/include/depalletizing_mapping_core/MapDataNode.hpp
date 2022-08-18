#ifndef DEPALLETIZING_MAPPING_CORE_MAPDATANODE_HPP
#define DEPALLETIZING_MAPPING_CORE_MAPDATANODE_HPP

#include <depalletizing_mapping_base/depalletizing_mapping_base.hpp>

#include "KMeans.hpp"
#include "Ransac.hpp"
#include "PlaneModel.hpp"

namespace depalletizing_mapping
{
    class MapDataNode : public depalletizing_mapping::DataNode
    {
    public:
        MapDataNode();
        MapDataNode(Ransac& ransac);
        ~MapDataNode();

        PlaneModel GetPlaneModel() const;
        void SetPlaneModel(PlaneModel& planeModel);
        Ransac GetRansac() const;
        void SetRansac(Ransac& ransac);
        KMeans GetKMeans() const;
        void SetKMeans(KMeans& kmeans);
        std::vector<PlaneModel> GetDetectedPlanes() const;
        void SetDetectedPlanes(std::vector<PlaneModel>& detectedPlanes);

        void RunRansac();
        void RunKMeansClustering();

    private:
        KMeans mKMeans;
        Ransac mRansac;
        PlaneModel mPlaneModel;
        std::vector<PlaneModel> mDetectedPlanes;
    };
}

#endif //DEPALLETIZING_MAPPING_CORE_MAPDATANODE_HPP
