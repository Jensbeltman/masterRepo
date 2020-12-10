#ifndef MASTER_GENETICEVALUATOROBJECTCANDIDATES_PPH
#define MASTER_GENETICEVALUATOROBJECTCANDIDATES_PPH

#include "GeneticEvaluator.hpp"
#include <vector>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <opencv2/viz/types.hpp>
#include <fcl/fcl.h>
#include <ga/typedefinitions.hpp>
#include <dataset/Dataset.hpp>
#include <chronometer.h>

typedef std::shared_ptr<cv::viz::Mesh> MeshPtr;

class GeneticEvaluatorOC : public GeneticEvaluator {
public:
    GeneticEvaluatorOC(DatasetObjectPtr datasetObjectPtr, int datapoint_n, double inlier_threshold = 1);
    void initialise_datapoint(int datapoint_n);

    DatasetObjectPtr datasetObjectPtr;
    DataPoint dp;
    PointCloudT::Ptr pc;//point cloud data
    PointCloudT::Ptr pcm;//mesh point cloud
    NormalCloudT::Ptr ncm;//mesh normal cloud
    std::shared_ptr<cv::viz::Mesh> meshPtr;
    pcl::KdTreeFLANN<PointT>::Ptr kdtree;
    T4 camera_pose;
    std::vector<T4> object_candidates;//object candidates
    //std::vector<pcl::IndicesPtr> oc_visible_pt_idxs;
    std::vector<pcl::IndicesPtr> oc_visible_inlier_pt_idxs;
    std::vector<PointCloudT::Ptr> visible_oc_pcs;
    std::vector<std::pair<int,int>>  oc_collision_pairs;

    double inlier_threshold;

    double evaluate_chromosome(chromosomeT &chromosome);

private:
    void init_visible_inliers();

    void init_collisions();

    Chronometer chronometer;
};

typedef std::shared_ptr<GeneticEvaluatorOC> GeneticEvaluatorOCPtr;

#endif //MASTER_GENETICEVALUATOROBJECTCANDIDATES_PPH
