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


typedef fcl::BVHModel<fcl::OBBRSSf> CollisionModel;
typedef std::shared_ptr<CollisionModel> CollisionModelPtr;
typedef std::shared_ptr<cv::viz::Mesh> MeshPtr;

class GeneticEvaluatorOC : public GeneticEvaluator {
public:
    GeneticEvaluatorOC(DatasetObjectPtr datasetObjectPtr, int sample_n, double inlier_threshold = 0.001);

    GeneticEvaluatorOC(std::vector<T4> &poses, PointCloudT::Ptr &pc, PointCloudT::Ptr &pcm, NormalCloudT::Ptr &ncm,
                       std::shared_ptr<cv::viz::Mesh> &meshptr, T4 &camera_pose, double inlier_threshold = 0.001);


    PointCloudT::Ptr pc;//point cloud data
    PointCloudT::Ptr pcm;//mesh point cloud
    NormalCloudT::Ptr ncm;//mesh normal cloud
    std::shared_ptr<cv::viz::Mesh> meshPtr;
    CollisionModelPtr collisionModelPtr;
    pcl::KdTreeFLANN<PointT>::Ptr kdtree;
    T4 camera_pose;
    std::vector<T4> object_candidates;//object candidates
    std::vector<pcl::IndicesPtr> oc_visible_pt_idxs;
    std::vector<pcl::IndicesPtr> oc_visible_inlier_pt_idxs;
    std::vector<std::vector<int>> oc_collision_idxs; // Todo might be more efficient to use bit_and on integer types however scaling might be an issue

    static CollisionModelPtr mesh_to_coll_model(MeshPtr meshptr);

    double inlier_threshold;

    double evaluate_chromosome(chromosomeT &chromosome);

private:
    void init();

    void init_visible_inliers();

    void init_collisions();

    Chronometer chronometer;
};


#endif //MASTER_GENETICEVALUATOROBJECTCANDIDATES_PPH
