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
    std::string pc_data_ext;
    std::string mesh_data_ext;
    std::filesystem::path path;
    std::filesystem::path mesh_path;
    std::filesystem::path mesh_pcd_path;
    std::vector<std::string> filenames;
    T4 camera_pose;


    explicit DatasetObject(std::filesystem::path path = ".", std::string data_ext = ".PCD");
    // VIRTUAL BASECLASS FUNCTIONS
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr get_pcd(int n);

    virtual std::shared_ptr<cv::viz::Mesh> get_mesh();

    virtual pcl::shared_ptr<PointCloudT> get_mesh_point_cloud();

    virtual pcl::shared_ptr<NormalCloudT> get_mesh_normal_cloud();

    virtual int size() const;

    virtual bool has_gt(int n);

    virtual bool has_scores(int n);

    virtual std::vector<T4> get_gt(unsigned int n);

    virtual std::vector<double> get_scores(unsigned int n);

    virtual std::vector<T4> get_object_candidates(unsigned int n);


protected:
    void get_filenames_with_ext_from_dir(std::string ext, std::string dir, std::vector<std::string> &vofs);
};

typedef std::shared_ptr<DatasetObject> DatasetObjectPtr;

#endif //MASTER_DATASETOBJECT_H
