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

    n_ocs=dp.ocs.size();
    mask.resize(n_ocs);
    std::fill(mask.begin(),mask.end(),true);
    mask_set =  false;
}

bool GeneticEvaluator::sanityCheck(chromosomeT &chromosome) {
    size_t chromosome_size = chromosome.size();
    if (n_ocs != chromosome_size) {
        std::cout << "Chromosome and ground truth poses are not same dimension returning 0.0 cost" << std::endl;
        return false;
    }
    return true;
}

void GeneticEvaluator::set_dp_mask(chromosomeT &new_mask){
    mask_set = true;
    mask = new_mask;
}


