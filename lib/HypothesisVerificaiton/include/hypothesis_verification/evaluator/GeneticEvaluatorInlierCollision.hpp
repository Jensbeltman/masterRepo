#ifndef MASTER_GENETICEVALUATOROBJECTCANDIDATES_PPH
#define MASTER_GENETICEVALUATOROBJECTCANDIDATES_PPH

#include "GeneticEvaluator.hpp"
#include "hypothesis_verification/evaluator/functionality/VisibleInlier.hpp"
#include "hypothesis_verification/evaluator/functionality/Collision.hpp"
#include <vector>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/viz/types.hpp>
#include <fcl/fcl.h>
#include "../typedefinitions.hpp"
#include <dataset/Dataset.hpp>
#include <chronometer.h>
#include "collision_checking.hpp"

class GeneticEvaluatorInlierCollision : public VisibleInlier, public Collision {
public:
    explicit GeneticEvaluatorInlierCollision(double inlier_threshold = 1.0, double sigmoid_falloff_center = 4.0,
                                             double sigmoid_falloff_scale = 2.0, double oc_inlier_multiplier = 0.5);

    void init_datapoint(DataPoint &datapoint) override;

    // Hyper params maps
    double sigmoid_center;
    double sigmoid_growth_rate;
    double oc_inlier_threshold;

    virtual double evaluate_chromosome(chromosomeT &chromosome) override;

    double sigmoid_fall_off(double x);
};

typedef std::shared_ptr<GeneticEvaluatorInlierCollision> GeneticEvaluatorInlierCollisionPtr;

#endif //MASTER_GENETICEVALUATOROBJECTCANDIDATES_PPH
