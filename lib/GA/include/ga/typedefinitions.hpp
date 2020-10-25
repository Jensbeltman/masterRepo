#ifndef MASTER_TYPEDEFINITIONS_H
#define MASTER_TYPEDEFINITIONS_H

#include <ostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef std::vector<bool> chromosomeT;
typedef std::vector<chromosomeT> populationT;

std::ostream &operator<<(std::ostream &os, chromosomeT chromosome);

std::ostream &operator<<(std::ostream &os, populationT population);

typedef Eigen::Transform<double, 3, Eigen::Affine> T4;
typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;

#endif //MASTER_TYPEDEFINITIONS_HPP
