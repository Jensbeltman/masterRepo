#ifndef MASTER_VIZUALIZATION_HPP
#define MASTER_VIZUALIZATION_HPP


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <dataset/Dataset.hpp>

pcl::visualization::PCLVisualizer::Ptr vis_pc_and_oc(DatasetObjectPtr datasetObject,int n){
    PointCloudT::Ptr pc = datasetObject->get_pcd(n);
    PointCloudT::Ptr meshpc = datasetObject->get_mesh_point_cloud();
    std::vector<T4> ocs = datasetObject->get_object_candidates(n);

    pcl::visualization::PCLVisualizer::Ptr vis = pcl::make_shared<pcl::visualization::PCLVisualizer>();
    vis->addPointCloud(pc, "data");

    for(int i = 0;i<ocs.size();i++){
        PointCloudT::Ptr ocpc = pcl::make_shared<PointCloudT>();
        pcl::transformPointCloud(*meshpc, *ocpc, ocs[i]);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(ocpc, 0, 255, 0);
        vis->addPointCloud<pcl::PointXYZ>(ocpc, color, "oc_" + std::to_string(i));
    }
    vis->spin();

    return vis;
}


#endif //MASTER_VIZUALIZATION_HPP
