#ifndef MASTER_VISIBLEINLIER_HPP
#define MASTER_VISIBLEINLIER_HPP
#include "hypothesis_verification/evaluator/GeneticEvaluator.hpp"

class VisibleInlier: virtual public GeneticEvaluator {
public:
    VisibleInlier(double nn_inlier_threshold = 1);
    void init_visible_inliers();

    std::vector<pcl::IndicesPtr> oc_visible_inlier_pt_idxs;
    std::vector<PointCloudT::Ptr> visible_oc_pcs;

    double nn_inlier_threshold;
};

#endif //MASTER_VISIBLEINLIER_HPP
