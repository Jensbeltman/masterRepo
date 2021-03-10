#ifndef MASTER_INTERSECTINGPOINTS_HPP
#define MASTER_INTERSECTINGPOINTS_HPP

#include "hypothesis_verification/evaluator/functionality/Collision.hpp"
#include "hypothesis_verification/evaluator/functionality/VisibleInlier.hpp"

class IntersectingPoints: public VisibleInlier,public Collision {
public:
    void init_intersecting_points();
    void get_max_intersection_in_chromosome(chromosomeT &chromosome,std::vector<int> &intersections);
    std::vector<int> collision_point_intersections;
};


#endif //MASTER_INTERSECTINGPOINTS_HPP
