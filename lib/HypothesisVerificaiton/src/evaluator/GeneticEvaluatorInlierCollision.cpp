#include "iostream"
#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollision.hpp"
#include "hypothesis_verification/evaluator/simple_visibility_function.hpp"
#include "hypothesis_verification/evaluator/collision_checking.hpp"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "chronometer.h"
#include <fcl/fcl.h>
#include "hypothesis_verification/evaluator/point_cloud_renderer.hpp"

using namespace std;


GeneticEvaluatorInlierCollision::GeneticEvaluatorInlierCollision(double inlier_threshold, double sigmoid_falloff_center,
                                                                 double sigmoid_falloff_scale, double oc_inlier_threshold)
        : nn_inlier_threshold(inlier_threshold),
          sigmoid_center(sigmoid_falloff_center),
          sigmoid_growth_rate(sigmoid_falloff_scale),
          oc_inlier_threshold(oc_inlier_threshold),
          voxelGridPtr(new pcl::VoxelGrid<PointT>){

    type = "GeneticEvaluatorOC";
}

GeneticEvaluatorInlierCollision::GeneticEvaluatorInlierCollision(DatasetObjectPtr datasetObjectPtr, int datapoint_n, double inlier_threshold,
                                                                 double sigmoid_falloff_center, double sigmoid_falloff_scale,
                                                                 double oc_inlier_threshold)
        : nn_inlier_threshold(inlier_threshold),
          sigmoid_center(sigmoid_falloff_center),
          sigmoid_growth_rate(sigmoid_falloff_scale),
          oc_inlier_threshold(oc_inlier_threshold),
          voxelGridPtr(new pcl::VoxelGrid<PointT>){

    type = "GEIC";
    initialise_object(datasetObjectPtr, datapoint_n);

}

void GeneticEvaluatorInlierCollision::initialise_object(DatasetObjectPtr &newDatasetObjectPtr, int datapoint_n) {
    initialise_object(newDatasetObjectPtr,newDatasetObjectPtr->data_points[datapoint_n]);
}

void GeneticEvaluatorInlierCollision::initialise_object(DatasetObjectPtr &newDatasetObjectPtr, DataPoint &datapoint) {
    datasetObjectPtr = newDatasetObjectPtr;
    pcm = datasetObjectPtr->get_mesh_point_cloud();

    ncm = datasetObjectPtr->get_mesh_normal_cloud();
    meshPtr = datasetObjectPtr->get_mesh();
    camera_pose = datasetObjectPtr->camera_pose;

    initialise_datapoint(datapoint);
}
void GeneticEvaluatorInlierCollision::initialise_datapoint(int datapoint_n) {
    initialise_datapoint(datasetObjectPtr->data_points[datapoint_n]);
}

void GeneticEvaluatorInlierCollision::initialise_datapoint(DataPoint &datapoint) {
    dp = datapoint;
    pc = datasetObjectPtr->get_pcd(dp);
    // KdTree of cloud data
    kdtree = pcl::make_shared<pcl::KdTreeFLANN<PointT>>();
    kdtree->setInputCloud(pc);
    init_visible_inliers();
    init_collisions();
}


void GeneticEvaluatorInlierCollision::init_visible_inliers() {
    int n_ocs = dp.ocs.size();
    oc_visible_inlier_pt_idxs.clear();
    chronometer.tic();
    PointCloudRenderer pc_render;
    pc_render.addActorsPLY(datasetObjectPtr->mesh_path, dp.ocs);
    pc_render.fitCameraAndResolution();
    visible_oc_pcs.clear();
    visible_oc_pcs.reserve(n_ocs);
    pc_render.renderPointClouds(visible_oc_pcs);

    double render_time = chronometer.toc();

    // Down sample
    voxelGridPtr->setLeafSize(vg_leaf_size,vg_leaf_size,vg_leaf_size);
    voxelGridPtr->setInputCloud(pc);
    voxelGridPtr->filter(*pc);
    for(auto &vpc:visible_oc_pcs){
        voxelGridPtr->setInputCloud(vpc);
        voxelGridPtr->filter(*vpc);
    }

    // Vectors for knn
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    oc_visible_inlier_pt_idxs.resize(n_ocs);
    for (int i = 0;i<n_ocs;i++) {
        auto &pc = visible_oc_pcs[i];

        pcl::IndicesPtr & inlier_pts = oc_visible_inlier_pt_idxs[i];
        inlier_pts = pcl::make_shared<pcl::Indices>();

        for (auto &p: pc->points) {
            kdtree->radiusSearch(p, nn_inlier_threshold, k_indices, k_sqr_distances,
                                 1);
            if (!k_indices.empty()) {
                inlier_pts->push_back(k_indices[0]);
            }
        }
    }
    std::cout << "Inliers time: " << chronometer.toc() << "s," << "Render Time: "
              << render_time << "s\n";
}

void GeneticEvaluatorInlierCollision::init_collisions() {
    collisions.pairs.clear();
    // Detect collisions
    chronometer.tic();
    collisions = get_collisions(dp.ocs, meshPtr);
    std::cout << "Collision init elapsed time: " << chronometer.toc() << "s\n";
}

double GeneticEvaluatorInlierCollision::evaluate_chromosome(chromosomeT &chromosome) {

    if (dp.ocs.size() != chromosome.size()) {
        std::cout << "Chromosome and ground truth poses are not same dimension returning 0.0 cost" << endl;
        return 0.0;
    }
    double cost = pc->size();
    int n_active_genes = 0;
    int vis_inlier_pt_cnt_tot = 0;
    int vis_pt_cnt_tot = 0;
    auto coll_pair_itt = collisions.pairs.begin();

    // Check if ocs are in collision
    std::vector<bool> in_collision;
    std::vector<double> penetration;
    get_collision(chromosome, in_collision, penetration);

    for (int i = 0; i < chromosome.size(); i++) {
        if (chromosome[i]) {
            int vis_inlier_pt_cnt = oc_visible_inlier_pt_idxs[i]->size();
            vis_pt_cnt_tot += visible_oc_pcs[i]->points.size();

            if (!in_collision[i]) {
                vis_inlier_pt_cnt_tot += vis_inlier_pt_cnt;
            } else {
                vis_inlier_pt_cnt_tot += sigmoid_fall_off(penetration[i]) * vis_inlier_pt_cnt;
            }
            n_active_genes++;
        }
    }
  // oc_inlier_threshold determins the fraction of vis_pt_cnt_tot/oc_inlier_threshold that creats zero extra cost. If this fraction is higher than oc_inlier_threshold cost decreases
    cost += (vis_pt_cnt_tot - (1.0 / oc_inlier_threshold) * vis_inlier_pt_cnt_tot);

    return cost;
}


double GeneticEvaluatorInlierCollision::sigmoid_fall_off(double x) {
    return 1 / (1 + std::exp(-sigmoid_growth_rate * (x - sigmoid_center)));
}

void GeneticEvaluatorInlierCollision::get_collision(chromosomeT &chromosome, vector<bool> &in_collision, vector<double> &penetration) {
    // This version calculated the maximum collision depth for a given oc
    in_collision.resize(chromosome.size(),false);
    penetration.resize(chromosome.size(),0);
    for (int i = 0; i < collisions.pairs.size(); i++){
        auto &cp = collisions.pairs[i];
        double &pen_dist = collisions.distances[i];
        if (chromosome[cp.first] && chromosome[cp.second]) {
            in_collision[cp.first] = true;
            in_collision[cp.second] = true;

            penetration[cp.first] = std::max(penetration[cp.first], pen_dist);
            penetration[cp.second] = std::max(penetration[cp.second], pen_dist);
        }
    }
}

