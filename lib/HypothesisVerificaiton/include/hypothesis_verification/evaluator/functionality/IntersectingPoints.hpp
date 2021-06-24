#ifndef MASTER_INTERSECTINGPOINTS_HPP
#define MASTER_INTERSECTINGPOINTS_HPP

#include "hypothesis_verification/evaluator/functionality/Collision.hpp"
#include "hypothesis_verification/evaluator/functionality/VisibleInlier.hpp"

class IntersectingPoints: public VisibleInlier,public Collision {
public:
    void init_intersecting_points();
    void init_object(DatasetObjectPtr &datasetObjectPtr) override;
    void get_max_intersection_in_chromosome(chromosomeT &chromosome,std::vector<int> &intersections);
    std::vector<int> collision_point_intersections;
    double mode_bounding_sphere_rad_squared;

};


#endif //MASTER_INTERSECTINGPOINTS_HPP
