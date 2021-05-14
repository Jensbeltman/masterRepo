#include "hypothesis_verification/evaluator/functionality/IntersectingPoints.hpp"

void IntersectingPoints::init_intersecting_points() {
    init_visible_inliers();
    init_collisions();
    collision_point_intersections.clear();
    n_ocs = dp.ocs.size();
    collision_point_intersections.resize(n_ocs*(n_ocs-1)/2,0);

    std::vector<int> already_in_collision;
    std::vector<int> v_intersection;
    int n_intersections=0;
    for(int i = 0; i<n_ocs;i++) {
        std::sort(oc_visible_inlier_pt_idxs[i]->begin(), oc_visible_inlier_pt_idxs[i]->end());
    }

    int pair_idx=0;
    for(int i = 0; i<(n_ocs-1);i++){
        for(int j =i+1; j<n_ocs;j++){
            // Find intersections and save the max count
            v_intersection.clear();
            std::set_intersection(oc_visible_inlier_pt_idxs[i]->begin(), oc_visible_inlier_pt_idxs[i]->end(),
                                  oc_visible_inlier_pt_idxs[j]->begin(),
                                  oc_visible_inlier_pt_idxs[j]->end(),
                                  std::back_inserter(v_intersection));

            collision_point_intersections[pair_idx++]=(static_cast<int>(v_intersection.size()));
        }
    }
   /* for (int i = 0; i < collisions.pairs.size(); i++) {
        auto &cp = collisions.pairs[i];

        // Sort indicies if the first time they are in a collision pair
        if (std::find(already_in_collision.begin(), already_in_collision.end(), cp.first) != already_in_collision.end())
            std::sort(oc_visible_inlier_pt_idxs[cp.first]->begin(), oc_visible_inlier_pt_idxs[cp.first]->end());
        if (std::find(already_in_collision.begin(), already_in_collision.end(), cp.second) !=
            already_in_collision.end())
            std::sort(oc_visible_inlier_pt_idxs[cp.second]->begin(), oc_visible_inlier_pt_idxs[cp.second]->end());

        // Find intersections and save the max count
        v_intersection.clear();
        std::set_intersection(oc_visible_inlier_pt_idxs[cp.first]->begin(), oc_visible_inlier_pt_idxs[cp.first]->end(),
                              oc_visible_inlier_pt_idxs[cp.second]->begin(),
                              oc_visible_inlier_pt_idxs[cp.second]->end(),
                              std::back_inserter(v_intersection));

        collision_point_intersections.emplace_back(static_cast<int>(v_intersection.size()));
    }*/
}

void IntersectingPoints::get_max_intersection_in_chromosome(chromosomeT &chromosome, std::vector<int> &intersections) {
    intersections.resize(chromosome.size(),0);
    int pair_idx=0;
    for(int i = 0; i<(chromosome.size()-1);i++){
        for(int j =i+1; j<chromosome.size();j++){
            if (chromosome[i] && chromosome[j]) {
                intersections[i] = std::max(intersections[i], collision_point_intersections[pair_idx]);
                intersections[j] = std::max(intersections[j], collision_point_intersections[pair_idx]);
            }
            pair_idx++;
        }
    }

    /*    intersections.resize(chromosome.size(),0);
    for (int i = 0; i < collisions.pairs.size(); i++){
        auto &cp = collisions.pairs[i];
        if (chromosome[cp.first] && chromosome[cp.second]) {
            intersections[cp.first] = std::max(intersections[cp.first], collision_point_intersections[i]);
            intersections[cp.second] = std::max(intersections[cp.second], collision_point_intersections[i]);
        }
    }*/
}
