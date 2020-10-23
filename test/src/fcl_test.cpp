#include <chronometer.h>
#include <dataset/sileane/SileaneDataset.hpp>
#include <dataset/scape/ScapeDataset.hpp>
#include <vector>
#include <utility>
#include <memory>
#include <opencv2/viz.hpp>
#include <fcl/fcl.h>
#include <dataset/vizualization.hpp>
#include <ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp>

typedef fcl::BVHModel<fcl::OBBRSSf> CollisionModel;
typedef std::shared_ptr<CollisionModel> CollisionModelPtr;
typedef std::shared_ptr<cv::viz::Mesh> MeshPtr;


/// @brief Collision data for use with the DefaultCollisionFunction. It stores
/// the collision request and the result given by collision algorithm (and
/// stores the conclusion of whether further evaluation of the broadphase
/// collision manager has been deemed unnecessary).
template <typename S>
struct CustomCollisionData {
    fcl::CollisionRequest<S> request;
    fcl::CollisionResult<S> result;
    std::vector<std::pair<fcl::CollisionObject<S>*,fcl::CollisionObject<S>*>> final_results;

    std::vector<bool> was_in_collision;
    /// If `true`, requests that the broadphase evaluation stop.
    bool done{false};
};


template <typename S>
bool CustomCollisionFunction(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2,
                              void* data) {
    assert(data != nullptr);
    auto* collision_data = static_cast<CustomCollisionData<S>*>(data);
    const fcl::CollisionRequest<S>& request = collision_data->request;
    fcl::CollisionResult<S>& result = collision_data->result;
    collide(o1, o2, request, result);


    collision_data->was_in_collision.emplace_back(result.isCollision());
    result.clear();
    if(collision_data->was_in_collision.back()) { collision_data->final_results.emplace_back(o1, o2); }
/*    std::cout<<"Distance: "<<(o1->getTranslation()-o2->getTranslation()).norm()<<"  "<<o1->getTranslation()[0]<<","<<o1->getTranslation()[1]<<","<<o1->getTranslation()[2]<<" <-> "<<o2->getTranslation()[0]<<","<<o2->getTranslation()[1]<<","<<o2->getTranslation()[2]<<std::endl;
    if (collision_data->was_in_collision.back())
    {
        std::cout<<"In collision "<<result.numContacts()<<std::endl;

    }else{
        std::cout<<"Not In collision"<<std::endl;
    }*/


    return collision_data->done;
}



std::vector<std::vector<int>> get_collisions(std::vector<T4> &object_candidates, std::vector<fcl::CollisionObjectf *> &collisionObjects,CustomCollisionData<float> &collision_data){
    std::vector<std::vector<int>> collisions(object_candidates.size());
    for(auto &p:collision_data.final_results){
        int first_obj_match = std::distance(collisionObjects.begin(),std::find(collisionObjects.begin(),collisionObjects.end(),p.first));
        int second_obj_match = std::distance(collisionObjects.begin(),std::find(collisionObjects.begin(),collisionObjects.end(),p.second));
        collisions[first_obj_match].push_back(second_obj_match);
    }
    return collisions;
}

std::ostream &operator<<(std::ostream &os, std::vector<std::vector<int>> collisions) {
    os << "Collisions:\n";
    for (int i = 0;i<collisions.size();i++) {
        os << i<<": ";
        for (auto &v:collisions[i]) {
            os << v << "\t";
        }
        os << "\n";
    }
    os << "\n";
    return os;
};



int main(){
    Chronometer chronometer;
/*    SileaneData sileaneData("/home/jens/masterData/Siléane-Dataset");
    DatasetObjectPtr datasetObject = sileaneData.get_object_by_name("bunny");
    SileaneDatasetObjectPtr sileaneObject = std::dynamic_pointer_cast<SileaneDatasetObject>(datasetObject);
    sileaneObject->n_noisy_poses=30; // To get gt*/


    ScapeDataset scapeData("/home/jens/masterData/ScapeDataset/Scape/Full Dataset",
                           "/home/jens/masterData/ScapeDataset/Data from Scape Recognition",false);
    DatasetObjectPtr datasetObject = scapeData.get_object_by_name("Ears");
    ScapeDatasetObjectPtr scapeObject = std::dynamic_pointer_cast<ScapeDatasetObject>(datasetObject);


    std::shared_ptr<cv::viz::Mesh> meshPtr = scapeObject->get_mesh();
    std::vector<T4> object_candidates = scapeObject->get_object_candidates(3);


//    // Create collision objects
    std::vector<fcl::Vector3f> vertices;
    std::vector<fcl::Triangle> triangles;

    // Extract mesh data from opencv mesh type
    for (auto itt = meshPtr->cloud.begin<cv::Vec3f>(); itt != meshPtr->cloud.end<cv::Vec3f>(); ++itt)
        vertices.push_back(fcl::Vector3f((*itt)[0], (*itt)[1], (*itt)[2]));
    for (auto itt = meshPtr->polygons.begin<cv::Vec3i>(); itt != meshPtr->polygons.end<cv::Vec3i>(); ++itt)
        triangles.push_back(fcl::Triangle((*itt)[0], (*itt)[1], (*itt)[2]));

    CollisionModelPtr collModelPtr = std::make_shared<CollisionModel>();

    // add the mesh data into the BVHModel structure
    collModelPtr->beginModel();
    collModelPtr->addSubModel(vertices, triangles);
    collModelPtr->endModel();

    // Generation of collision matrix
   // oc_collision_idxs.resize(object_candidates.size());

    fcl::DynamicAABBTreeCollisionManagerf *manager = new fcl::DynamicAABBTreeCollisionManagerf;
    manager->setup();

    chronometer.tic();
    std::vector<fcl::CollisionObjectf *> collisionObjects;
    for (int i = 0; i < object_candidates.size(); i++) {
        fcl::CollisionObjectf* collisionObject = new fcl::CollisionObjectf(collModelPtr);
        collisionObject->setTranslation(object_candidates[i].matrix().block<3,1>(0,3).cast<float>());
        collisionObject->setRotation(object_candidates[i].matrix().block<3,3>(0,0).cast<float>());
        collisionObjects.emplace_back(collisionObject);
    }

    manager->registerObjects(collisionObjects);

    CustomCollisionData<float> custom_collision_data;

    manager->collide(&custom_collision_data, CustomCollisionFunction);
    std::cout<<"Number of ocs: "<<object_candidates.size()<<std::endl;

    std::cout<<custom_collision_data.was_in_collision<<std::endl;
    std::cout << "Collision elapsed time: " << chronometer.toc() << "s\n";

    cout<<get_collisions(object_candidates,collisionObjects,custom_collision_data);

    pcl::visualization::PCLVisualizer::Ptr vis = vis_pc_and_oc(datasetObject,3);
    vis->spin();
    while (!vis->wasStopped ())
    {
        vis->spinOnce ();
    }

    return 0;
}