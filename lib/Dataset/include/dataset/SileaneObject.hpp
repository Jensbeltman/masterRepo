#ifndef OPENGACUSTOM_SILEANEOBJECT_HPP
#define OPENGACUSTOM_SILEANEOBJECT_HPP

#include <iostream>
#include "SileaneCameraParams.hpp"
#include "SileaneDepthToPCD.hpp"
#include <string>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <nlohmann/json.hpp>
#include <typeinfo>
#include <array>
#include <Eigen/Eigen>
#include <opencv2/viz/vizcore.hpp>
#include "ga/utility/typedefinitions.hpp"
#include <dataset/DatasetObject.hpp>


class SileaneObject : public DatasetObject {
public:
    SileaneCameraParams camera_params;
    bool has_depth_gt;


    SileaneObject(std::filesystem::path path = ".", std::string data_ext = ".PNG") : DatasetObject(path, data_ext) {
        type = "Sileane";
        get_filenames_with_ext_from_dir(data_ext, path, filenames);
        mesh_path = (path / "mesh").replace_extension(".ply");
        mesh_pcd_path = (path / "mesh").replace_extension(".pcd");
        camera_params = SileaneCameraParams(path / "camera_params.txt");
        has_depth_gt = std::filesystem::exists(path / "depth_gt");
        if (!std::filesystem::exists(path / "depth")) {
            std::cout << "No depth directory found object" << "\"name\"" << "might not work as intended" << std::endl;
        }
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr get_pcd(int n) { return get_pcd(n, false); }

    pcl::PointCloud<pcl::PointXYZ>::Ptr get_pcd(int n, bool gt) {
        if (n < filenames.size()) {
            std::filesystem::path dir = path;
            if (gt) {
                if (has_depth_gt) {
                    dir /= "depth_gt";
                } else {
                    std::cout << "No gt data available" << std::endl;
                }
            } else {
                dir /= "depth";
            }
            return sileane_depth_to_pcd((dir / filenames[n]).replace_extension(data_ext), camera_params);
        } else {
            return nullptr;
        }
    };

    cv::Mat get_color(int n, bool gt = false) {
        if (n < filenames.size()) {
            std::filesystem::path p = (path / "rgb" / filenames[n]).replace_extension(data_ext);
            if (std::filesystem::exists(p)) {
                return cv::imread(p, cv::IMREAD_UNCHANGED);
            } else {
                std::cout << "Could not find color image at \"" << p << "\"" << std::endl;
            }
        }
        return cv::Mat();
    };


    std::vector<T4> get_gt_poses(unsigned int n) {
        std::ifstream i((path / "gt" / filenames[n]).replace_extension(".json"));
        nlohmann::json j;
        i >> j;

        std::vector<T4> gt_poses;
        for (auto gt : j) {
            T4 T;
            T(0, 0) = gt["R"][0][0];
            T(0, 1) = gt["R"][0][1];
            T(0, 2) = gt["R"][0][2];
            T(1, 0) = gt["R"][1][0];
            T(1, 1) = gt["R"][1][1];
            T(1, 2) = gt["R"][1][2];
            T(2, 0) = gt["R"][2][0];
            T(2, 1) = gt["R"][2][1];
            T(2, 2) = gt["R"][2][2];
            T(0, 3) = gt["t"][0];
            T(1, 3) = gt["t"][1];
            T(2, 3) = gt["t"][2];
            gt_poses.push_back(T);
        }
        return gt_poses;
    }

};

#endif //OPENGACUSTOM_SILEANEOBJECT_HPP
