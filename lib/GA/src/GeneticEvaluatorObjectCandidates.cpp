#include "iostream"
#include "ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp"
#include "ga/point_cloud_processing/point_cloud_processing.hpp"
#include <ga/collision/collision_checking.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "chronometer.h"
#include <fcl/fcl.h>


using namespace std;

GeneticEvaluatorOC::GeneticEvaluatorOC(DatasetObjectPtr doPtr, int sample_n, double inlier_threshold)
        : inlier_threshold(inlier_threshold) {

    pc = doPtr->get_pcd(sample_n);
    pcm = doPtr->get_mesh_point_cloud();
    ncm = doPtr->get_mesh_normal_cloud();
    meshPtr = doPtr->get_mesh();
    object_candidates = doPtr->get_object_candidates(sample_n);
    camera_pose = doPtr->camera_pose;
    init();
}

GeneticEvaluatorOC::GeneticEvaluatorOC(std::vector<T4> &poses, PointCloudT::Ptr &pc, PointCloudT::Ptr &pcm,
                                       NormalCloudT::Ptr &ncm, std::shared_ptr<cv::viz::Mesh> &meshptr,
                                       T4 &camera_pose,
                                       double
                                       inlier_threshold) : pc(pc), pcm(pcm), ncm(ncm), meshPtr(meshptr),
                                                           object_candidates(poses),
                                                           inlier_threshold(inlier_threshold),
                                                           camera_pose(camera_pose) {
    init();
}


void GeneticEvaluatorOC::init() {
    type = "GeneticEvaluatorOC";

    // KdTree of cloud data
    std::cout << (pc->size() > 0) << std::endl;
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
    // Detect collisions
    oc_collision_pairs = get_collisions(object_candidates,meshPtr);
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

