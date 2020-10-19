#ifndef MASTER_DATASETOBJECT_H
#define MASTER_DATASETOBJECT_H

#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/viz/vizcore.hpp>
#include <dataset/typedefinitions.hpp>


typedef Eigen::Transform<double, 3, Eigen::Affine> T4;

class DatasetObject {
public:
    std::string name;
    std::string type;
    std::filesystem::path path;
    std::filesystem::path mesh_path;
    std::filesystem::path mesh_pcd_path;
    std::string pc_data_ext;
    std::string mesh_data_ext;
    std::vector<std::string> filenames;
    T4 camera_pose;


    DatasetObject(std::filesystem::path path = ".", std::string data_ext = ".PCD")
            : path(path), pc_data_ext(data_ext) {
        name = path.stem().string();
    };

    // VIRTUAL BASECLASS FUNCTIONS
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr get_pcd(int n){return nullptr;};

    virtual std::shared_ptr<cv::viz::Mesh> get_mesh() {
        std::shared_ptr<cv::viz::Mesh> meshptr = std::make_shared<cv::viz::Mesh>(cv::viz::Mesh::load(mesh_path));
        return meshptr;
    }

    virtual pcl::shared_ptr<PointCloudT> get_mesh_point_cloud() {
        pcl::shared_ptr<PointCloudT> pc = pcl::make_shared<PointCloudT>();
        pcl::io::loadPCDFile(mesh_pcd_path, *pc);
        return pc;
    }

    virtual pcl::shared_ptr<NormalCloudT> get_mesh_normal_cloud() {
        pcl::shared_ptr<NormalCloudT> nc = pcl::make_shared<NormalCloudT>();
        pcl::io::loadPCDFile(mesh_pcd_path, *nc);
        return nc;
    }

    virtual int size() const { return filenames.size(); };

    virtual std::vector<T4> get_object_candidates(unsigned int n) { return std::vector<T4>();};

    // UTILITY FUNCTIONS


protected:
    void get_filenames_with_ext_from_dir(std::string ext, std::string dir, std::vector<std::string> &vofs) {
        for (auto &p : std::filesystem::directory_iterator(dir)) {
            if (p.path().extension() == ext)
                vofs.push_back(p.path().stem().string());
        }
    };
};


#endif //MASTER_DATASETOBJECT_H
