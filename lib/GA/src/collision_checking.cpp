#include "ga/collision/collision_checking.hpp"
#include "fcl/narrowphase/collision.h"
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <utility>

struct CustomCollisionData {
    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;
    std::vector<std::pair<fcl::CollisionObject<double>*,fcl::CollisionObject<double>*>> final_results;

    bool done{false};
};

bool CustomCollisionFunction(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2,
                             void* data) {
    assert(data != nullptr);
    auto* collision_data = static_cast<CustomCollisionData*>(data);
    const fcl::CollisionRequest<double>& request = collision_data->request;
    fcl::CollisionResult<double>& result = collision_data->result;

    collide(o1, o2, request, result);
    if(result.isCollision()) { collision_data->final_results.emplace_back(o1, o2); }

    result.clear();
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
    if(n_vert)
        std::cout<<n_vert<<" faces where not tris"<<std::endl;



    CollisionModelPtr collModelPtr = std::make_shared<CollisionModel>();

    // add the mesh data into the BVHModel structure
    collModelPtr->beginModel();
    collModelPtr->addSubModel(vertices, triangles);
    collModelPtr->endModel();

    return collModelPtr;
}

std::vector<std::pair<int,int>> get_collisions(std::vector<T4> &object_candidates, MeshPtr &meshPtr){
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
    std::vector<std::pair<int,int>> collisions;
    int first_obj_match, second_obj_match;
    for(auto &p:custom_collision_data.final_results){
        first_obj_match = std::distance(collisionObjects.begin(),std::find(collisionObjects.begin(),collisionObjects.end(),p.first));
        second_obj_match = std::distance(collisionObjects.begin(),std::find(collisionObjects.begin(),collisionObjects.end(),p.second));
        collisions.emplace_back(first_obj_match,second_obj_match);
    }

    std::sort(collisions.begin(),collisions.end());

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