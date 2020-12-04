#include "ga/point_cloud_processing/point_cloud_processing.hpp"

namespace pp{

   pcl::IndicesPtr get_visible_indices(NormalCloudT::Ptr &nc, T4 &camera_transform) {
        unsigned int i = 0;
       pcl::IndicesPtr indices(new pcl::Indices);
        Eigen::Vector3f nv;
        Eigen::Vector3f zv = camera_transform.rotation().matrix().col(2).cast<float>();
        for (auto &n:nc->points) {
            nv = n.getNormalVector3fMap();
            if (zv.dot(nv) < 0.0)
                indices->push_back(i);
            i++;
        }

        return indices;
    }

    PointCloudT::Ptr get_visible_point_cloud(PointCloudT::Ptr &pc, NormalCloudT::Ptr &nc, T4 &camera_transform) {

        assert(pc->size() == nc->size());

        PointCloudT::Ptr vispc=pcl::make_shared<PointCloudT>();
        vispc->reserve(pc->size()/2); // Gues on amount of visible points
        Eigen::Vector3f nv;
        Eigen::Vector3f zv = camera_transform.rotation().matrix().col(2).cast<float>();

        for (int i = 0;i<nc->size();i++) {
            nv = nc->points[i].getNormalVector3fMap();
            if (zv.dot(nv) < 0.0)
                vispc->emplace_back(pc->points[i]);
        }

        return vispc;
    }

}