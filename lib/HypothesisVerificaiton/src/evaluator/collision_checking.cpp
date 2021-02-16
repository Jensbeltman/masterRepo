#include "hypothesis_verification/evaluator/collision_checking.hpp"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/distance.h"
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <utility>

struct CustomCollisionData {
    const fcl::CollisionRequest<double> coll_request{1,true};
    fcl::CollisionResult<double> coll_result;

    std::vector<std::pair<fcl::CollisionObject<double>*,fcl::CollisionObject<double>*>> collision_pairs;
    std::vector<double> collision_pair_penetration_depth;

    bool done{false};
};

bool CustomCollisionFunction(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2,
                             void* data) {
    assert(data != nullptr);
    auto* collision_data = static_cast<CustomCollisionData*>(data);
    const fcl::CollisionRequest<double>& coll_request = collision_data->coll_request;
    fcl::CollisionResult<double>& coll_result = collision_data->coll_result;


    collide(o1, o2, coll_request, coll_result);
    if(coll_result.isCollision()) {
        collision_data->collision_pairs.emplace_back(o1, o2);
        collision_data->collision_pair_penetration_depth.emplace_back(coll_result.getContact(0).penetration_depth);
    }

    coll_result.clear();
    return collision_data->done;
}

CollisionModelPtr get_coll_model(MeshPtr meshptr) {
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;
    // Extract mesh data from opencv mesh type
    for (auto itt = meshptr->cloud.begin<cv::Vec3f>(); itt != meshptr->cloud.end<cv::Vec3f>(); ++itt)
        vertices.emplace_back((*itt)[0], (*itt)[1], (*itt)[2]);

    auto itt = meshptr->polygons.begin<int32_t>();
    auto end = meshptr->polygons.end<int32_t>();
    int non_tris = 0;
    int n_vert=0;
    int32_t a,b,c;
    while (itt != end) {
        n_vert = *itt++;
        if (n_vert!=3){
            non_tris++;
            itt+=n_vert;
        }
        else {
            a=*itt++;
            b=*itt++;
            c=*itt++;
            triangles.emplace_back(a,b,c);
        }
    }
    if(non_tris)
        std::cout<<non_tris<<" faces where not tris"<<std::endl;



    CollisionModelPtr collModelPtr = std::make_shared<CollisionModel>();

    // add the mesh data into the BVHModel structure
    collModelPtr->beginModel();
    collModelPtr->addSubModel(vertices, triangles);
    collModelPtr->endModel();

    return collModelPtr;
}

Collisions get_collisions(std::vector<T4> &object_candidates, MeshPtr &meshPtr){
    CollisionModelPtr collisionModelPtr = get_coll_model(meshPtr);

    auto *manager = new fcl::DynamicAABBTreeCollisionManager<double>;
    manager->setup();

    // Fill collision manager with transformed collision models
    std::vector<fcl::CollisionObject<double> *> collisionObjects;
    for (auto & object_candidate : object_candidates) {
        auto* collisionObject = new fcl::CollisionObject<double>(collisionModelPtr);
        collisionObject->setTranslation(object_candidate.matrix().block<3,1>(0,3));
        collisionObject->setRotation(object_candidate.matrix().block<3,3>(0,0));
        collisionObjects.emplace_back(collisionObject);
    }
    manager->registerObjects(collisionObjects);

    // Do self collision check
    CustomCollisionData custom_collision_data;
    manager->collide(&custom_collision_data, CustomCollisionFunction);

    // Get collision pair index
    Collisions collisions;
    int first_obj_match, second_obj_match;
    for(auto &p:custom_collision_data.collision_pairs){
        first_obj_match = std::distance(collisionObjects.begin(),std::find(collisionObjects.begin(),collisionObjects.end(),p.first));
        second_obj_match = std::distance(collisionObjects.begin(),std::find(collisionObjects.begin(),collisionObjects.end(),p.second));
        collisions.pairs.emplace_back(first_obj_match, second_obj_match);
    }

    collisions.distances = custom_collision_data.collision_pair_penetration_depth;

    std::sort(collisions.pairs.begin(), collisions.pairs.end());

    delete manager;
    for(auto& p:collisionObjects)
        delete p;
    return collisions;
}

std::ostream &operator<<(std::ostream &os, const std::vector<std::pair<int,int>>& collisions) {
    os << "Collisions:\n";
    for (auto & p : collisions) {
            os << p.first << ","<<p.second<<"\n";
    }
    os << "\n";
    return os;
};