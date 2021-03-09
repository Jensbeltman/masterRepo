#include "hypothesis_verification/evaluator/functionality/IntersectingPoints.hpp"

void IntersectingPoints::init_intersecting_points() {
    init_visible_inliers();
    init_collisions();
    collision_point_intersections.clear();

    std::vector<int> already_in_collision;
    std::vector<int> v_intersection;
    int n_intersections=0;
    for (int i = 0; i < collisions.pairs.size(); i++) {
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
    }
}
