#ifndef MASTER_COLLISION_CHECK_HPP
#define MASTER_COLLISION_CHECK_HPP

#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision_object.h>
#include <ga/typedefinitions.hpp>
#include <opencv2/viz/types.hpp>

typedef fcl::BVHModel<fcl::OBBRSSd> CollisionModel;
typedef std::shared_ptr<CollisionModel> CollisionModelPtr;
typedef std::shared_ptr<cv::viz::Mesh> MeshPtr;

std::vector<std::pair<int,int>> get_collisions(std::vector<T4> &object_candidates, MeshPtr &meshPtr);
std::ostream &operator<<(std::ostream &os, const std::vector<std::pair<int,int>>& collisions);

#endif //MASTER_COLLISION_CHECK_HPP
