#include "iostream"
#include "ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp"
#include "ga/point_cloud_processing/point_cloud_processing.hpp"
#include <ga/collision/collision_checking.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "chronometer.h"
#include <fcl/fcl.h>

using namespace std;

GeneticEvaluatorOC::GeneticEvaluatorOC(DatasetObjectPtr datasetObjectPtr, int datapoint_n, double inlier_threshold)
        : inlier_threshold(inlier_threshold), datasetObjectPtr(datasetObjectPtr),
          pcm(datasetObjectPtr->get_mesh_point_cloud()), ncm(datasetObjectPtr->get_mesh_normal_cloud()),
          meshPtr(datasetObjectPtr->get_mesh()), camera_pose(datasetObjectPtr->camera_pose) {

    initialise_datapoint(datapoint_n);

    type = "GeneticEvaluatorOC";
}

void GeneticEvaluatorOC::initialise_datapoint(int datapoint_n) {
    dp = datasetObjectPtr->data_points[datapoint_n];
    pc = datasetObjectPtr->get_pcd(datapoint_n);
    // KdTree of cloud data
    kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointT>>();
    kdtree->setInputCloud(pc);
    init_visible_inliers();
    init_collisions();
}

//Todo consider if this should be a seperate function
void GeneticEvaluatorOC::init_visible_inliers() {
    oc_visible_pt_idxs.clear();
    oc_visible_inlier_pt_idxs.clear();
    chronometer.tic();
    // Vectors for knn
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;

    for (T4 oc:dp.ocs) {
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
    oc_collision_pairs.clear();
    // Detect collisions
    chronometer.tic();
    oc_collision_pairs = get_collisions(dp.ocs, meshPtr);
    std::cout << "Collision init elapsed time: " << chronometer.toc() << "s\n";
}

double GeneticEvaluatorOC::evaluate_chromosome(chromosomeT &chromosome) {

    if (dp.ocs.size() != chromosome.size()) {
        std::cout << "Chromosome and ground truth poses are not same dimension returning 0.0 cost" << endl;
        return 0.0;
    }
    double cost = 0;
    int n_active_genes = 0;
    int vis_inlier_pt_cnt_tot = 0;
    int vis_pt_cnt_tot = 0;
    auto coll_pair_itt = oc_collision_pairs.begin();

    // Check if ocs are in collision
    std::vector<bool> in_collision(chromosome.size(), false);
    for (auto &cp:oc_collision_pairs) {
        if (chromosome[cp.first] && chromosome[cp.second]) {
            in_collision[cp.first] = true;
            in_collision[cp.second] = true;
        }
    }

    for (int i = 0; i < dp.ocs.size(); i++) {
        if (chromosome[i]) {
            int vis_inlier_pt_cnt = oc_visible_inlier_pt_idxs[i]->size();
            int vis_pt_cnt = oc_visible_pt_idxs[i]->size();

            if (in_collision[i]) {
                vis_inlier_pt_cnt_tot -= vis_inlier_pt_cnt;
            } else {
                vis_inlier_pt_cnt_tot += vis_inlier_pt_cnt;
                n_active_genes++;
            }
            vis_pt_cnt_tot += vis_pt_cnt;


        }
    }


    cost = -vis_inlier_pt_cnt_tot;


    if (isnan(cost) || !n_active_genes)
        cost = std::numeric_limits<double>::max();

    return cost;
}

