#ifndef MASTER_POINT_CLOUD_PROCESSING_HPP
#define MASTER_POINT_CLOUD_PROCESSING_HPP

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include "ga/typedefinitions.hpp"

namespace pp {
    pcl::IndicesPtr get_visible_indices(NormalCloudT::Ptr &nc, T4 &camera_transform);
    PointCloudT::Ptr get_visible_point_cloud(PointCloudT::Ptr &pc, NormalCloudT::Ptr &nc, T4 &camera_transform);
}
#endif //MASTER_POINT_CLOUD_PROCESSING_HPP
