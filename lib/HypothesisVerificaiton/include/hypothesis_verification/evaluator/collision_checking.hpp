#ifndef MASTER_COLLISION_CHECK_HPP
#define MASTER_COLLISION_CHECK_HPP

#include <fcl/geometry/bvh/BVH_model.h>

#include "../typedefinitions.hpp"
#include <opencv2/viz/types.hpp>

typedef fcl::BVHModel<fcl::OBBRSSd> CollisionModel;
typedef std::shared_ptr<CollisionModel> CollisionModelPtr;
typedef std::shared_ptr<cv::viz::Mesh> MeshPtr;

struct Collisions{
    std::vector<std::pair<int,int>> pairs;
    std::vector<double> distances;
};

CollisionModelPtr get_coll_model(MeshPtr meshptr);
Collisions get_collisions(std::vector<T4> &object_candidates, MeshPtr &meshPtr);
std::ostream &operator<<(std::ostream &os, const std::vector<std::pair<int,int>>& collisions);

#endif //MASTER_COLLISION_CHECK_HPP
