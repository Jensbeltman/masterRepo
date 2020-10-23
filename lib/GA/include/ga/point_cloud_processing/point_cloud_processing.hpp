#ifndef MASTER_POINT_CLOUD_PROCESSING_HPP
#define MASTER_POINT_CLOUD_PROCESSING_HPP

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include "ga/typedefinitions.hpp"

namespace pp {
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
    };
}
#endif //MASTER_POINT_CLOUD_PROCESSING_HPP
