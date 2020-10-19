#ifndef OPENGACUSTOM_SILEANEOBJECT_HPP
#define OPENGACUSTOM_SILEANEOBJECT_HPP

#include <iostream>
#include <string>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <vector>
#include <array>
#include <typeinfo>
#include <nlohmann/json.hpp>
#include <Eigen/Eigen>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <dataset/pose_noise.hpp>
#include "SileaneCameraParams.hpp"
#include <dataset/DatasetObject.hpp>
#include "dataset/typedefinitions.hpp"


class SileaneDatasetObject : public DatasetObject {
public:
    SileaneCameraParams sileaneCameraParams;
    double mu_noise = 0.01;
    double sigma_noise = M_PI / 4;
    int n_noisy_poses = -1;
    bool has_depth_gt;


    SileaneDatasetObject(std::filesystem::path path = ".", std::string data_ext = ".PNG") : DatasetObject(path,
                                                                                                          data_ext) {
        type = "Sileane";
        pc_data_ext = ".pcd";
        mesh_data_ext = ".ply";
        get_filenames_with_ext_from_dir(data_ext, path, filenames);
        mesh_path = (path / "mesh").replace_extension(mesh_data_ext);
        mesh_pcd_path = (path / "mesh").replace_extension(pc_data_ext);
        sileaneCameraParams = SileaneCameraParams(path / "camera_params.txt");
        camera_pose = sileaneCameraParams.T;
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
            return sileane_depth_to_pcd((dir / filenames[n]).replace_extension(pc_data_ext), sileaneCameraParams);
        } else {
            return nullptr;
        }
    };

    cv::Mat get_color(int n, bool gt = false) {
        if (n < filenames.size()) {
            std::filesystem::path p = (path / "rgb" / filenames[n]).replace_extension(pc_data_ext);
            if (std::filesystem::exists(p)) {
                return cv::imread(p, cv::IMREAD_UNCHANGED);
            } else {
                std::cout << "Could not find color image at \"" << p << "\"" << std::endl;
            }
        }
        return cv::Mat();
    };


    std::vector<T4> get_object_candidates(unsigned int n) {
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

        PoseNoise poseNoise(mu_noise, sigma_noise);
        if (n_noisy_poses == -1)
            poseNoise.append_noisy_transforms(gt_poses, gt_poses.size());
        else
            poseNoise.append_noisy_transforms(gt_poses, n_noisy_poses);

        return gt_poses;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr sileane_depth_to_pcd(std::string path, SileaneCameraParams &camera_params) {
        cv::Mat depth = cv::imread(path, cv::IMREAD_UNCHANGED);
        int width = depth.cols;
        int height = depth.rows;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
        auto pc_itt = pc->begin();
        cv::MatIterator_<u_int16_t> depth_itt = depth.begin<u_int16_t>();
        double depth_range = camera_params.clip_end - camera_params.clip_start;
        pcl::PointXYZ p;

        for (int r = 0; r < height; r++) {
            for (int c = 0; c < width; c++) {
                u_int16_t d = depth.at<u_int16_t>(r, c);
                if (d < 65535) {
                    pcl::PointXYZ p;
                    p.z = camera_params.clip_start + depth_range * (d / (float) 65535);
                    p.x = p.z * (c - camera_params.cu) / (double) camera_params.fu;
                    p.y = p.z * (r - camera_params.cv) / (double) camera_params.fv;
                    pc->push_back(p);
                }
            }
        }
        pcl::transformPointCloud(*pc, *pc, camera_params.location, camera_params.rotation);

        return pc;
    }

};

#endif //OPENGACUSTOM_SILEANEOBJECT_HPP
