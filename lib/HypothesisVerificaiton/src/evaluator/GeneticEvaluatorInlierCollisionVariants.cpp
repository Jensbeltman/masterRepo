#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollisionVariants.hpp"

GeneticEvaluatorInlierCollisionScaled::GeneticEvaluatorInlierCollisionScaled() {
    type="GEICS";
}

double GeneticEvaluatorInlierCollisionScaled::evaluate_chromosome(chromosomeT &chromosome) {
    if (dp.ocs.size() != chromosome.size()) {
        std::cout << "Chromosome and ground truth poses are not same dimension returning 0.0 cost" << std::endl;
        return 0.0;
    }
    double n_pc_points = pc->size();
    int n_active_genes = 0;
    int vis_inlier_pt_cnt_tot = 0;
    int vis_pt_cnt_tot = 0;

    // Check if ocs are in collision
    std::vector<bool> in_collision(chromosome.size(), false);
    std::vector<double> penetration(chromosome.size(), 0);
    get_collision(chromosome, in_collision, penetration);

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

    double cost = 1 - (vis_inlier_pt_cnt_tot / (n_pc_points + std::max(oc_inlier_threshold*vis_pt_cnt_tot - vis_inlier_pt_cnt_tot,0.0)));

    return cost;
}

GeneticEvaluatorScoreCollision::GeneticEvaluatorScoreCollision() {
    type = "GESC";
}
void GeneticEvaluatorScoreCollision::initialise_datapoint(DataPoint &datapoint) {
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
    get_collision(chromosome, in_collision, penetration);

    double total_score = 0;
    for (int i = 0; i < chromosome.size(); i++) {
        if (chromosome[i]) {
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

void GeneticEvaluatorUniqueInlierCollisionScaled::initialise_datapoint(DataPoint &datapoint) {
    GeneticEvaluatorInlierCollision::initialise_datapoint(datapoint);

    n_point_intersections.clear();
    n_point_intersections.resize(dp.ocs.size(),0);

    std::vector<int> already_in_collision;
    std::vector<int> v_intersection;
    int n_intersections=0;
    for (int i = 0; i < collisions.pairs.size(); i++) {
        auto &cp = collisions.pairs[i];

        // Sort indicies if the first time they are in a collision pair
        if(std::find(already_in_collision.begin(),already_in_collision.end(),cp.first)!=already_in_collision.end())
            std::sort(oc_visible_inlier_pt_idxs[cp.first]->begin(), oc_visible_inlier_pt_idxs[cp.first]->end());
        if(std::find(already_in_collision.begin(),already_in_collision.end(),cp.second)!=already_in_collision.end())
            std::sort(oc_visible_inlier_pt_idxs[cp.second]->begin(), oc_visible_inlier_pt_idxs[cp.second]->end());

        // Find intersections and save the max count
        v_intersection.clear();
        std::set_intersection(oc_visible_inlier_pt_idxs[cp.first]->begin(), oc_visible_inlier_pt_idxs[cp.first]->end(),
                              oc_visible_inlier_pt_idxs[cp.second]->begin(), oc_visible_inlier_pt_idxs[cp.second]->end(),
                              std::back_inserter(v_intersection));
        n_intersections=static_cast<int>(v_intersection.size());
        n_point_intersections[cp.first] = std::max(n_intersections,n_point_intersections[cp.first]);
        n_point_intersections[cp.second] = std::max(n_intersections,n_point_intersections[cp.second]);
    }
}

double GeneticEvaluatorUniqueInlierCollisionScaled::evaluate_chromosome(chromosomeT &chromosome) {
    if (dp.ocs.size() != chromosome.size()) {
        std::cout << "Chromosome and ground truth poses are not same dimension returning 0.0 cost" << std::endl;
        return 0.0;
    }
    double n_pc_points = pc->size();
    int n_active_genes = 0;
    int vis_inlier_pt_cnt_tot = 0;
    int n_point_intersections_tot = 0;
    int vis_pt_cnt_tot = 0;

    // Check if ocs are in collision
    std::vector<bool> in_collision(chromosome.size(), false);
    std::vector<double> penetration(chromosome.size(), 0);
    get_collision(chromosome, in_collision, penetration);

    for (int i = 0; i < chromosome.size(); i++) {
        if (chromosome[i]) {
            int vis_inlier_pt_cnt = oc_visible_inlier_pt_idxs[i]->size();
            vis_pt_cnt_tot += visible_oc_pcs[i]->points.size();

            if (!in_collision[i]) {
                vis_inlier_pt_cnt_tot += vis_inlier_pt_cnt;
            } else {
                vis_inlier_pt_cnt_tot += sigmoid_fall_off(penetration[i]) * vis_inlier_pt_cnt;
                n_point_intersections_tot += n_point_intersections[i];
            }
            n_active_genes++;
        }
    }

    double cost = 1 - ((vis_inlier_pt_cnt_tot-n_point_intersections_tot) / (n_pc_points + std::max(oc_inlier_threshold*vis_pt_cnt_tot - vis_inlier_pt_cnt_tot,0.0)));

    return cost;
}
