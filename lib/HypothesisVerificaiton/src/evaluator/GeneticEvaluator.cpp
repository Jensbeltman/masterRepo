#include "hypothesis_verification/evaluator/GeneticEvaluator.hpp"

GeneticEvaluator::GeneticEvaluator():voxelGridPtr(new pcl::VoxelGrid<PointT>) {}

void GeneticEvaluator::init(DatasetObjectPtr &datasetObjectPtr, int datapoint_n) {
    init(datasetObjectPtr, datasetObjectPtr->data_points[datapoint_n]);
}

void GeneticEvaluator::init(DatasetObjectPtr &datasetObjectPtr, DataPoint &datapoint) {
    init_object(datasetObjectPtr);
    init_datapoint(datapoint);
}

void GeneticEvaluator::init_datapoint(int datapoint_n) {
    init_datapoint(datasetObjectPtr->data_points[datapoint_n]);
}

void GeneticEvaluator::init_object(DatasetObjectPtr &newdatasetObjectPtr) {
    datasetObjectPtr = newdatasetObjectPtr;
    pcm = datasetObjectPtr->get_mesh_point_cloud();
    voxelGridPtr->setLeafSize(vg_leaf_size,vg_leaf_size,vg_leaf_size);
    voxelGridPtr->setInputCloud(pcm);
    voxelGridPtr->filter(*pcm);

    meshPtr = datasetObjectPtr->get_mesh();
}

void GeneticEvaluator::init_datapoint(DataPoint &datapoint) {
    dp = datapoint;
    pc = datasetObjectPtr->get_pcd(dp);

    // Voxel grid and build tree of captured pointcloud
    voxelGridPtr->setLeafSize(vg_leaf_size,vg_leaf_size,vg_leaf_size);
    voxelGridPtr->setInputCloud(pc);
    voxelGridPtr->filter(*pc);
    kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointT>>();
    kdtree->setInputCloud(pc);
}


