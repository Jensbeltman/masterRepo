#include "iostream"
#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollision.hpp"
#include "hypothesis_verification/evaluator/simple_visibility_function.hpp"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "chronometer.h"
#include <fcl/fcl.h>
#include "hypothesis_verification/evaluator/point_cloud_renderer.hpp"

using namespace std;


GeneticEvaluatorInlierCollision::GeneticEvaluatorInlierCollision(double inlier_threshold, double sigmoid_falloff_center,
                                                                 double sigmoid_falloff_scale, double oc_inlier_threshold)
        : VisibleInlier(inlier_threshold),
          sigmoid_center(sigmoid_falloff_center),
          sigmoid_growth_rate(sigmoid_falloff_scale),
          oc_inlier_threshold(oc_inlier_threshold){

    type = "GEIC";
}


void GeneticEvaluatorInlierCollision::init_datapoint(DataPoint &datapoint) {
    GeneticEvaluator::init_datapoint(datapoint);
    init_visible_inliers();
    init_collisions();
}

double GeneticEvaluatorInlierCollision::evaluate_chromosome(chromosomeT &chromosome) {

    if (dp.ocs.size() != chromosome.size()) {
        std::cout << "Chromosome and ground truth poses are not same dimension returning 0.0 cost" << endl;
        return 0.0;
    }
    double cost = pc->size();
    int n_active_genes = 0;
    int vis_inlier_pt_cnt_tot = 0;
    int vis_pt_cnt_tot = 0;
    auto coll_pair_itt = collisions.pairs.begin();

    // Check if ocs are in collision
    std::vector<bool> in_collision;
    std::vector<double> penetration;
    get_max_collisions_in_chromosome(chromosome, in_collision, penetration);

    for (int i = 0; i < chromosome.size(); i++) {
        if (chromosome[i]) {
            int vis_inlier_pt_cnt = oc_visible_inlier_pt_idxs[i]->size();
            vis_pt_cnt_tot += visible_oc_pcs[i]->points.size();

            if (!in_collision[i]) {
                vis_inlier_pt_cnt_tot += vis_inlier_pt_cnt;
            } else {
                vis_inlier_pt_cnt_tot += sigmoid_fall_off(penetration[i]) * vis_inlier_pt_cnt;
            }
            n_active_genes++;
        }
    }
  // oc_inlier_threshold determins the fraction of vis_pt_cnt_tot/oc_inlier_threshold that creats zero extra cost. If this fraction is higher than oc_inlier_threshold cost decreases
    cost += (vis_pt_cnt_tot - (1.0 / oc_inlier_threshold) * vis_inlier_pt_cnt_tot);

    return cost;
}


double GeneticEvaluatorInlierCollision::sigmoid_fall_off(double x) {
    return 1 / (1 + std::exp(-sigmoid_growth_rate * (x - sigmoid_center)));
}