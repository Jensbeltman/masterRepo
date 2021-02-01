#ifndef MASTER_GENETICEVALUATOROBJECTCANDIDATES_PPH
#define MASTER_GENETICEVALUATOROBJECTCANDIDATES_PPH

#include "GeneticEvaluator.hpp"
#include <vector>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/viz/types.hpp>
#include <fcl/fcl.h>
#include <ga/typedefinitions.hpp>
#include <dataset/Dataset.hpp>
#include <chronometer.h>
#include <ga/collision/collision_checking.hpp>

typedef std::shared_ptr<cv::viz::Mesh> MeshPtr;

class GeneticEvaluatorOC : public GeneticEvaluator {
public:
    explicit GeneticEvaluatorOC(double inlier_threshold = 1.0, double sigmoid_falloff_center = 4.0,
                                double sigmoid_falloff_scale = 2.0, double oc_inlier_multiplier = 0.5);

    GeneticEvaluatorOC(DatasetObjectPtr datasetObjectPtr, int datapoint_n, double inlier_threshold = 1.0,
                       double sigmoid_falloff_center = 4.0, double sigmoid_falloff_scale = 2.0,
                       double oc_inlier_multiplier = 0.5);

    void initialise_object(DatasetObjectPtr &datasetObjectPtr, int datapoint_n = 0) override;
    void initialise_object(DatasetObjectPtr &datasetObjectPtr, DataPoint &datapoint_n) override;

    void initialise_datapoint(int datapoint_n) override;
    void initialise_datapoint(DataPoint &datapoint_n) override;



    NormalCloudT::Ptr ncm;//mesh normal cloud
    std::shared_ptr<cv::viz::Mesh> meshPtr;
    pcl::KdTreeFLANN<PointT>::Ptr kdtree;
    T4 camera_pose;
    std::vector<T4> object_candidates;//object candidates
    //std::vector<pcl::IndicesPtr> oc_visible_pt_idxs;
    std::vector<pcl::IndicesPtr> oc_visible_inlier_pt_idxs;
    std::vector<PointCloudT::Ptr> visible_oc_pcs;
    Collisions collisions;
    std::vector<std::pair<int, int>> oc_collision_pairs;
    std::vector<double> oc_collision_pair_penetration;

    // Hyper params maps
    pcl::VoxelGrid<PointT>::Ptr voxelGridPtr;
    double nn_inlier_threshold;
    double sigmoid_falloff_center;
    double sigmoid_falloff_scale;
    double oc_inlier_threshold;
    double vg_leaf_size;

    virtual double evaluate_chromosome(chromosomeT &chromosome) override;

    double sigmoid_fall_off(double x);


private:
    void init_visible_inliers();

    void init_collisions();

    Chronometer chronometer;
};

typedef std::shared_ptr<GeneticEvaluatorOC> GeneticEvaluatorOCPtr;

#endif //MASTER_GENETICEVALUATOROBJECTCANDIDATES_PPH
