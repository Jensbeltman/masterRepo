#ifndef MASTER_SILEANEDEPTHTOPCD_HPP
#define MASTER_SILEANEDEPTHTOPCD_HPP

#include <iostream>
#include <string>
#include "SileaneCameraParams.hpp"
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <opencv4/opencv2/imgcodecs.hpp>


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


#endif //MASTER_SILEANEDEPTHTOPCD_HPP
