#include "ga/GeneticEvaluatorObjectCandidates.hpp"
#include "iostream"
#include "ga/utility/point_cloud_processing.hpp"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "chronometer.h"
#include <fcl/fcl.h>


using namespace std;

GeneticEvaluatorOC::GeneticEvaluatorOC(DatasetObjectPtr doPtr, int sample_n, double inlier_threshold) :inlier_threshold(inlier_threshold){

    pc = doPtr->get_pcd(sample_n);
    pcm = doPtr->get_mesh_point_cloud();
    ncm = doPtr->get_mesh_normal_cloud();
    meshPtr = doPtr->get_mesh();
    object_candidates = doPtr->get_gt_poses(sample_n);
    camera_pose = doPtr->camera_pose;
    init();
}

GeneticEvaluatorOC::GeneticEvaluatorOC(std::vector<T4> &poses, PointCloudT::Ptr &pc, PointCloudT::Ptr &pcm,
                                       NormalCloudT::Ptr &ncm, std::shared_ptr<cv::viz::Mesh> &meshptr,
                                       T4 &camera_pose,
                                       double
                                       inlier_threshold) : pc(pc), pcm(pcm), ncm(ncm),meshPtr(meshptr), object_candidates(poses),
                                                           inlier_threshold(inlier_threshold),
                                                           camera_pose(camera_pose) {
    init();
}


void GeneticEvaluatorOC::init() {
    type = "GeneticEvaluatorOC";

    // KdTree of cloud data
    std::cout<<(pc->size() > 0)<<std::endl;
    kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointT>>();
    kdtree->setInputCloud(pc);

    init_visible_inliers();
    //init_collisions();
}


//Todo consider if this should be a seperate function
void GeneticEvaluatorOC::init_visible_inliers() {
    chronometer.tic();
    // Vectors for knn
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;

    for (T4 oc:object_candidates) {
        // Generate visible idxs for object candidate
        T4 cameraMeshVis = (camera_pose.inverse() *
                            oc).inverse();// instead of transforming the point cloud i transform the camera for use in visiblity calc
        oc_visible_pt_idxs.push_back(pp::get_visible_indices(ncm,
                                                             cameraMeshVis));// get visible mesh point cloud indices based on normal information and camera pose

        // Generate visible inlier points between object candidate pc and data
        PointT p;
        pcl::detail::Transformer<float> tf(oc.matrix().cast<float>());
        pcl::IndicesPtr inlier_pts(new pcl::Indices);
        for (int &pi : *(oc_visible_pt_idxs.back())) {
            tf.se3(pcm->points[pi].data, p.data);
            kdtree->radiusSearch(p, inlier_threshold, k_indices, k_sqr_distances,
                                 1);//Todo speed might be improved by implementing a nn instead of knn too avoid vector usage
            if (!k_indices.empty()) {
                inlier_pts->push_back(k_indices[0]);
            }
        }
        oc_visible_inlier_pt_idxs.push_back(inlier_pts);
    }
    std::cout << "Inliers and visiblity init elapsed time: " << chronometer.toc() << "s\n";
}

void GeneticEvaluatorOC::init_collisions() {
    // Create collision objects
    collisionModelPtr = mesh_to_coll_model(meshPtr);

    chronometer.tic();
    // Generation of collision matrix
    oc_collision_idxs.resize(object_candidates.size());

    fcl::BroadPhaseCollisionManagerf *manager = new fcl::DynamicAABBTreeCollisionManagerf();
    manager->setup();

    vector<fcl::CollisionObjectf *> collisionObjects;
    for (int i = 0; i < object_candidates.size(); i++) {
        collisionObjects.push_back(new fcl::CollisionObjectf(collisionModelPtr, fcl::Transform3f(
                object_candidates[i].matrix().cast<float>())));
    }
    cout << endl;
    manager->registerObjects(collisionObjects);

    fcl::DefaultCollisionData<float> collision_data;
    fcl::DefaultDistanceData<float> distance_data;

    manager->collide(&collision_data, fcl::DefaultCollisionFunction);

    std::vector<fcl::Contact<float>> contacts;
    collision_data.result.getContacts(contacts);
    cout << contacts.size() << endl;
    for (auto &val:contacts) {
        cout << val.pos.x() << " " << val.pos.z() << " " << val.pos.z() << " | ";
        //        auto it1 = find(collisionObjects.begin(),collisionObjects.end(), val.o1);
        //        auto it2 = find(collisionObjects.begin(),collisionObjects.end(), val.o2);
        //        if(it1!=collisionObjects.end()&&it2!=collisionObjects.end()){
        //            cout<<"("<<distance(collisionObjects.begin(),it1)<<","<<distance(collisionObjects.begin(),it1)<<") ";
        //        }
    }

    std::cout << std::endl;

    //    for(int i = object_candidates.size()-1;i >= 0; i--){
    //        for(int j = i-1;j >= 0; j--){
    //        fcl::CollisionRequestf request;
    //        fcl::CollisionResultf result;
    //        collide(collObjs[i].get(), collObjs[j].get(), request, result);
    //        oc_collision_idxs[i].push_back(j);
    //        }
    //    }

    std::cout << "Collision init elapsed time: " << chronometer.toc() << "s\n";
}

double GeneticEvaluatorOC::evaluate_chromosome(chromosomeT &chromosome) {

    if (object_candidates.size() != chromosome.size()) {
        std::cout << "Chromosome and ground truth poses are not same dimension returning 0.0 cost" << endl;
        return 0.0;
    }
    double cost = 0;
    int n_active_genes = 0;
    int vis_inlier_pt_cnt_tot = 0;
    int vis_pt_cnt_tot = 0;
    for (int i = 0; i < object_candidates.size(); i++) {
        if (chromosome[i]) {
            int vis_inlier_pt_cnt = oc_visible_inlier_pt_idxs[i]->size();
            int vis_pt_cnt = oc_visible_pt_idxs[i]->size();

            vis_inlier_pt_cnt_tot += vis_inlier_pt_cnt;
            vis_pt_cnt_tot += vis_pt_cnt;
            n_active_genes++;

        }
    }
    cost = 0.2 * (pc->points.size() - vis_inlier_pt_cnt_tot) / (float) n_active_genes +
           (vis_pt_cnt_tot - vis_inlier_pt_cnt_tot) / (float) n_active_genes;


    if (isnan(cost) || !n_active_genes)
        cost = std::numeric_limits<double>::max();

    return cost;
}

CollisionModelPtr GeneticEvaluatorOC::mesh_to_coll_model(MeshPtr meshptr) {
    std::vector<fcl::Vector3f> vertices;
    std::vector<fcl::Triangle> triangles;

    // Extract mesh data from opencv mesh type
    for (auto itt = meshptr->cloud.begin<cv::Vec3f>(); itt != meshptr->cloud.end<cv::Vec3f>(); ++itt)
        vertices.push_back(fcl::Vector3f((*itt)[0], (*itt)[1], (*itt)[2]));
    for (auto itt = meshptr->polygons.begin<cv::Vec3i>(); itt != meshptr->polygons.end<cv::Vec3i>(); ++itt)
        triangles.push_back(fcl::Triangle((*itt)[0], (*itt)[1], (*itt)[2]));

    std::shared_ptr<CollisionModel> collModelPtr = std::make_shared<CollisionModel>();

    // add the mesh data into the BVHModel structure
    collModelPtr->beginModel();
    collModelPtr->addSubModel(vertices, triangles);
    collModelPtr->endModel();


    return collModelPtr;
}

