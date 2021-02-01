#ifndef MASTER_SIMPLE_VISIBILITY_FUNCTION_HPP
#define MASTER_SIMPLE_VISIBILITY_FUNCTION_HPP

#include <pcl/common/common.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include "../typedefinitions.hpp"

namespace pp {
    pcl::IndicesPtr get_visible_indices(NormalCloudT::Ptr &nc, T4 &camera_transform);
    PointCloudT::Ptr get_visible_point_cloud(PointCloudT::Ptr &pc, NormalCloudT::Ptr &nc, T4 &camera_transform);
}
#endif //MASTER_SIMPLE_VISIBILITY_FUNCTION_HPP
