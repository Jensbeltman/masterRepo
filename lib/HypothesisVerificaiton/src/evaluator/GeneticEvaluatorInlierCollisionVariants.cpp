#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollisionVariants.hpp"
#include "dataset/transform_utility.hpp"

GeneticEvaluatorInlierCollisionScaled::GeneticEvaluatorInlierCollisionScaled() {
    type="GEICS";
}

double GeneticEvaluatorInlierCollisionScaled::evaluate_chromosome(chromosomeT &chromosome) {
    if (!sanityCheck(chromosome))
        return 0.0;

    double n_pc_points = pc->size();
    int n_active_genes = 0;
    int vis_inlier_pt_cnt_tot = 0;
    int vis_pt_cnt_tot = 0;

    // Check if ocs are in collision
    std::vector<bool> in_collision(chromosome.size(), false);
    std::vector<double> penetration(chromosome.size(), 0);
    get_max_collisions_in_chromosome(chromosome, in_collision, penetration);

    for (int i = 0; i < chromosome.size(); i++) {
        if (chromosome[i] && mask[i]) {
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

    double cost = (n_pc_points + (oc_inlier_threshold*vis_pt_cnt_tot - vis_inlier_pt_cnt_tot))/n_pc_points;

    return cost;
}

GeneticEvaluatorScoreCollision::GeneticEvaluatorScoreCollision() {
    type = "GESC";
}
void GeneticEvaluatorScoreCollision::init_datapoint(DataPoint &datapoint) {
    dp = datapoint;
    pc = datasetObjectPtr->get_pcd(dp);
    // KdTree of cloud data
    kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointT>>();
    kdtree->setInputCloud(pc);
    init_collisions();
}

double GeneticEvaluatorScoreCollision::evaluate_chromosome(chromosomeT &chromosome) {
    // Check if ocs are in collision
    std::vector<bool> in_collision(chromosome.size(), false);
    std::vector<double> penetration(chromosome.size(), 0);
    get_max_collisions_in_chromosome(chromosome, in_collision, penetration);

    double total_score = 0;
    for (int i = 0; i < chromosome.size(); i++) {
        if (chromosome[i] && mask[i]) {
            if (!in_collision[i]) {
                total_score += dp.oc_scores[i];
            } else {
                total_score += 2 * (0.5 - sigmoid_fall_off(penetration[i])) * dp.oc_scores[i];
            }
        }
    }
    // oc_inlier_threshold determins the fraction of vis_pt_cnt_tot/oc_inlier_threshold that creats zero extra cost. If this fraction is higher than oc_inlier_threshold cost decreases
    double cost = -total_score;

    return cost;
}


GeneticEvaluatorUniqueInlierCollisionScaled::GeneticEvaluatorUniqueInlierCollisionScaled() {
    type = "GEUICS";
}

void GeneticEvaluatorUniqueInlierCollisionScaled::init_datapoint(DataPoint &datapoint) {
    GeneticEvaluator::init_datapoint(datapoint);
    init_intersecting_points();
}

double GeneticEvaluatorUniqueInlierCollisionScaled::evaluate_chromosome(chromosomeT &chromosome) {
    if (!sanityCheck(chromosome))
        return 0.0;

    double n_pc_points = pc->size();
    int n_active_genes = 0;
    int vis_inlier_pt_cnt_tot = 0;
    int n_point_intersections_tot = 0;
    int vis_pt_cnt_tot = 0;

    // Check if ocs are in collision
    std::vector<bool> in_collision(chromosome.size(), false);
    std::vector<double> penetration(chromosome.size(), 0);
    std::vector<int> intersecting_inliers(chromosome.size(), 0);
    get_max_collisions_in_chromosome(chromosome, in_collision, penetration);
    get_max_intersection_in_chromosome(chromosome,intersecting_inliers);
    {
        for (int i = 0; i < chromosome.size(); i++) {
            n_point_intersections_tot += inlier_overlap_penalty_factor * intersecting_inliers[i];
        }
    }

    for (int i = 0; i < chromosome.size(); i++) {
        if (chromosome[i] && mask[i]) {
            vis_pt_cnt_tot += visible_oc_pcs[i]->points.size();
            vis_inlier_pt_cnt_tot +=  oc_visible_inlier_pt_idxs[i]->size();
            n_active_genes++;
        }
    }


    double cost =  (n_pc_points + (oc_inlier_threshold*vis_pt_cnt_tot - (vis_inlier_pt_cnt_tot-n_point_intersections_tot)))/n_pc_points;

    return cost;
}

GeneticEvaluatorLR::GeneticEvaluatorLR() {
    type = "GELR";
}

void GeneticEvaluatorLR::init_datapoint(DataPoint &datapoint) {
    GeneticEvaluator::init_datapoint(datapoint);
    init_intersecting_points();
}

double GeneticEvaluatorLR::evaluate_chromosome(chromosomeT &chromosome) {
    if (!sanityCheck(chromosome))
        return 0.0;

    std::vector<bool> inCollision;
    std::vector<double> penetration;
    std::vector<int> intersections;
    get_max_collisions_in_chromosome(chromosome,inCollision,penetration);
    get_max_intersection_in_chromosome(chromosome,intersections);

    double model_points =pcm->size();
    double cost=0;
    for (int i = 0; i < chromosome.size(); i++) {
        if(chromosome[i] && mask[i]) {
            double visiblePoints = visible_oc_pcs[i]->size();
            double visibleInliers = oc_visible_inlier_pt_idxs[i]->size();

            double x = dp.oc_scores[i] * score_w +
                       (visiblePoints / model_points) * visiblePointsFrac_w +
                       (visibleInliers / visiblePoints) * visibleInlierFrac_w +
                       penetration[i] * penetration_w +
                       (intersections[i] / visibleInliers) * intersectingInliersFrac_w
                       + intercept;
            cost -=int((1.0/(1.0+std::exp(-x))) >= 0.5)-0.5;;
        }
    }
    return cost;
}

GeneticEvaluatorLRC::GeneticEvaluatorLRC() {
    type = "GELRC";
}

double GeneticEvaluatorLRC::evaluate_chromosome(chromosomeT &chromosome) {
    if (!sanityCheck(chromosome))
        return 0.0;

    std::vector<bool> inCollision;
    std::vector<double> penetration;
    std::vector<int> intersections;
    get_max_collisions_in_chromosome(chromosome,inCollision,penetration);
    get_max_intersection_in_chromosome(chromosome,intersections);

    double model_points =pcm->size();
    double cost=0;
    for (int i = 0; i < chromosome.size(); i++) {
        if(chromosome[i] && mask[i]) {
            double visiblePoints = visible_oc_pcs[i]->size();
            double visibleInliers = oc_visible_inlier_pt_idxs[i]->size();

            double x = dp.oc_scores[i] * score_w +
                       (visiblePoints / model_points) * visiblePointsFrac_w +
                       (visibleInliers / visiblePoints) * visibleInlierFrac_w +
                       penetration[i] * penetration_w +
                       (intersections[i] / visibleInliers) * intersectingInliersFrac_w
                       + intercept;
            cost -= std::round(1.0/(1.0+std::exp(-x)))-0.5;;
        }
    }
    return cost;
}

GeneticEvaluatorLRS::GeneticEvaluatorLRS() {
    type = "GELRS";

}

double GeneticEvaluatorLRS::evaluate_chromosome(chromosomeT &chromosome) {
    chromosomeT chromosomeFull(chromosome.size(),true);
    if (!sanityCheck(chromosome))
        return 0.0;

    std::vector<bool> inCollision;
    std::vector<double> penetration;
    std::vector<int> intersections;
    get_max_collisions_in_chromosome(chromosomeFull,inCollision,penetration);
    get_max_intersection_in_chromosome(chromosomeFull,intersections);

    double model_points =pcm->size();
    double cost=0;
    for (int i = 0; i < chromosome.size(); i++) {
        if(chromosome[i] && mask[i]) {
            double visiblePoints = visible_oc_pcs[i]->size();
            double visibleInliers = oc_visible_inlier_pt_idxs[i]->size();

            double x = dp.oc_scores[i] * score_w +
                       (visiblePoints / model_points) * visiblePointsFrac_w +
                       (visibleInliers / visiblePoints) * visibleInlierFrac_w +
                       penetration[i] * penetration_w +
                       (intersections[i] / visibleInliers) * intersectingInliersFrac_w
                       + intercept;
            cost -= int((1.0/(1.0+std::exp(-x))) >= 0.5)-0.5;
        }
    }
    return cost;
}




GeneticEvaluatorF1::GeneticEvaluatorF1() {
    type = "GEF1";
}

void GeneticEvaluatorF1::init_datapoint(DataPoint &datapoint) {
    dp = datapoint;
    pc = datasetObjectPtr->get_pcd(dp);
    tu::find_correct_ocs(dp.ocs, dp.gts, t_thresh, r_thresh, correct_oc_indices, datasetObjectPtr->symmetry_transforms);
}

double GeneticEvaluatorF1::evaluate_chromosome(chromosomeT &chromosome) {
    tp = 0;
    tn = 0;
    fp = 0;
    fn = 0;
    tu::getFPTN(tp,tn,fp,fn,chromosome,correct_oc_indices);
    if(tp)
        return 1.0 - (static_cast<double>(2*tp) / static_cast<double>(2*tp + fp + fn));
    else if(tn==chromosome.size())
        return 0;
    else
        return 1;
}

GeneticEvaluatorPrecision::GeneticEvaluatorPrecision() {
    type = "GEPrecision";
}

double GeneticEvaluatorPrecision::evaluate_chromosome(chromosomeT &chromosome) {
    std::vector<int> correct_oc_indices;
    tu::find_correct_ocs(dp.ocs, dp.gts, t_thresh, r_thresh, correct_oc_indices, datasetObjectPtr->symmetry_transforms);
    tp = 0;
    tn = 0;
    fp = 0;
    fn = 0;
    tu::getFPTN(tp,tn,fp,fn,chromosome,correct_oc_indices);
    if(tp)
        return 1.0 - (static_cast<double>( tp) / static_cast<double>(tp + fp));
    else
        return 1;
}


