#ifndef MASTER_GENETICEVALUATOR_HPP
#define MASTER_GENETICEVALUATOR_HPP

#include "../typedefinitions.hpp"
#include "dataset/Dataset.hpp"
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <chronometer.h>

typedef std::shared_ptr<cv::viz::Mesh> MeshPtr;
class GeneticEvaluator {
public:
    GeneticEvaluator();

    std::string type = "GeneticEvaluator";

    DataPoint dp;
    DatasetObjectPtr datasetObjectPtr;
    int n_ocs=0;
    bool mask_set = false;

    chromosomeT mask;
    int active_mask_size=0;

    PointCloudT::Ptr pc;//point cloud data
    PointCloudT::Ptr pcm;//mesh point cloud
    std::shared_ptr<cv::viz::Mesh> meshPtr;

    pcl::KdTreeFLANN<PointT>::Ptr kdtree;
    pcl::VoxelGrid<PointT>::Ptr voxelGridPtr;
    double vg_leaf_size;

    void init(DatasetObjectPtr &datasetObjectPtr, int datapoint_n = 0);
    void init(DatasetObjectPtr &datasetObjectPtr, DataPoint &datapoint);
    void init_datapoint(int datapoint_n);
    void set_dp_mask(chromosomeT &mask);
    bool sanityCheck(chromosomeT &chromosome);

    virtual double evaluate_chromosome(chromosomeT &chromosome) = 0;
    virtual void init_object(DatasetObjectPtr &datasetObjectPtr);
    virtual void init_datapoint(DataPoint &datapoint);


protected:
    Chronometer chronometer;
};
typedef std::shared_ptr<GeneticEvaluator> GeneticEvaluatorPtr;


#endif //MASTER_GENETICEVALUATOR_HPP
