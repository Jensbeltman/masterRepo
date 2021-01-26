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
#include <ga/collision/collision_checking.hpp>

typedef std::shared_ptr<cv::viz::Mesh> MeshPtr;

class GeneticEvaluatorOC : public GeneticEvaluator {
public:
    explicit GeneticEvaluatorOC(double inlier_threshold = 1.0, double sigmoid_falloff_center = 4.0,
                                double sigmoid_falloff_scale = 2.0, double oc_inlier_multiplier = 0.5);

    GeneticEvaluatorOC(DatasetObjectPtr datasetObjectPtr, int datapoint_n, double inlier_threshold = 1.0,
                       double sigmoid_falloff_center = 4.0, double sigmoid_falloff_scale = 2.0,
                       double oc_inlier_multiplier = 0.5);

    void initialise_object(DatasetObjectPtr &datasetObjectPtr, int datapoint_n = 0);
    void initialise_object(DatasetObjectPtr &datasetObjectPtr, DataPoint &datapoint_n);

    void initialise_datapoint(int datapoint_n);
    void initialise_datapoint(DataPoint &datapoint_n);

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
    Collisions collisions;
    std::vector<std::pair<int, int>> oc_collision_pairs;
    std::vector<double> oc_collision_pair_penetration;

    // Hyper params maps
    std::map<std::string, int> hyper_params_i;
    std::map<std::string, double> hyper_params_d;
    double nn_inlier_threshold;
    double sigmoid_falloff_center;
    double sigmoid_falloff_scale;
    double oc_inlier_threshold;


    virtual double evaluate_chromosome(chromosomeT &chromosome);

    virtual double evaluate_chromosome(chromosomeT &chromosome, std::vector<double> coefficients);

    virtual void getHyperParameters_d(std::vector<std::string> &names, std::vector<double *> &params);

    virtual void setHyperParameters_d(std::vector<double> &params);

    double sigmoid_fall_off(double x);


private:
    void init_visible_inliers();

    void init_collisions();

    Chronometer chronometer;
};

typedef std::shared_ptr<GeneticEvaluatorOC> GeneticEvaluatorOCPtr;

#endif //MASTER_GENETICEVALUATOROBJECTCANDIDATES_PPH
