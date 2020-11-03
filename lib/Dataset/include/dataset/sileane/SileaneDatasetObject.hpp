#ifndef OPENGACUSTOM_SILEANEOBJECT_HPP
#define OPENGACUSTOM_SILEANEOBJECT_HPP

#include "dataset/typedefinitions.hpp"
#include "dataset/DatasetObject.hpp"
#include "dataset/sileane/SileaneCameraParams.hpp"
#include "dataset/transform_utility.hpp"

#include <iostream>
#include <string>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <vector>
#include <array>
#include <typeinfo>

#include <Eigen/Eigen>
#include <nlohmann/json.hpp>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/viz/vizcore.hpp>


class SileaneDatasetObject : public DatasetObject {
public:
    SileaneCameraParams sileaneCameraParams;
    double mu_noise = 0.01;
    double sigma_noise = M_PI / 4;
    int n_noisy_poses = -1;
    bool has_depth_gt;

    SileaneDatasetObject(std::filesystem::path path = ".", std::string data_ext = ".PNG");

    pcl::PointCloud<pcl::PointXYZ>::Ptr get_pcd(int n);

    pcl::PointCloud<pcl::PointXYZ>::Ptr get_pcd(int n, bool gt);

    cv::Mat get_color(int n, bool gt = false);

    std::vector<T4> get_object_candidates(unsigned int n);

    pcl::PointCloud<pcl::PointXYZ>::Ptr sileane_depth_to_pcd(std::string path, SileaneCameraParams &camera_params);

};

#endif //OPENGACUSTOM_SILEANEOBJECT_HPP
