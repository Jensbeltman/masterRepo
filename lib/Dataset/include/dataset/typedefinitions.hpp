#ifndef MASTER_TYPEDEFINITIONS_HPP
#define MASTER_TYPEDEFINITIONS_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef Eigen::Transform<double, 3, Eigen::Affine> T4;
typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;

#endif //MASTER_TYPEDEFINITIONS_HPP
