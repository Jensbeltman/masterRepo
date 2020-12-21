#include "iostream"
#include "ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp"
#include "ga/point_cloud_processing/point_cloud_processing.hpp"
#include <ga/collision/collision_checking.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "chronometer.h"
#include <fcl/fcl.h>
#include "ga/point_cloud_processing/point_cloud_renderer.hpp"

using namespace std;


GeneticEvaluatorOC::GeneticEvaluatorOC(double inlier_threshold, double sigmoid_falloff_center,
                                       double sigmoid_falloff_scale, double oc_inlier_threshold)
        : nn_inlier_threshold(inlier_threshold),
          sigmoid_falloff_center(sigmoid_falloff_center),
          sigmoid_falloff_scale(sigmoid_falloff_scale),
          oc_inlier_threshold(oc_inlier_threshold){
    type = "GeneticEvaluatorOC";
}

GeneticEvaluatorOC::GeneticEvaluatorOC(DatasetObjectPtr datasetObjectPtr, int datapoint_n, double inlier_threshold,
                                       double sigmoid_falloff_center, double sigmoid_falloff_scale,
                                       double oc_inlier_threshold)
        : nn_inlier_threshold(inlier_threshold),
          sigmoid_falloff_center(sigmoid_falloff_center),
          sigmoid_falloff_scale(sigmoid_falloff_scale),
          oc_inlier_threshold(oc_inlier_threshold){

    type = "GeneticEvaluatorOC";
    initialise_object(datasetObjectPtr, datapoint_n);
}

void GeneticEvaluatorOC::initialise_object(DatasetObjectPtr &newDatasetObjectPtr, int datapoint_n) {
    datasetObjectPtr = newDatasetObjectPtr;
    pcm = datasetObjectPtr->get_mesh_point_cloud();
    ncm = datasetObjectPtr->get_mesh_normal_cloud();
    meshPtr = datasetObjectPtr->get_mesh();
    camera_pose = datasetObjectPtr->camera_pose;

    initialise_datapoint(datapoint_n);
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


void GeneticEvaluatorOC::init_visible_inliers() {
    oc_visible_inlier_pt_idxs.clear();
    chronometer.tic();
    PointCloudRenderer pc_render;
    pc_render.addActorsPLY(datasetObjectPtr->mesh_path, dp.ocs);
    pc_render.fitCameraAndResolution();
    visible_oc_pcs.clear();
    visible_oc_pcs.reserve(dp.ocs.size());
    pc_render.renderPointClouds(visible_oc_pcs);
    double render_time = chronometer.toc();

    // Vectors for knn
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;

    for (auto &pc:visible_oc_pcs) {
        pcl::IndicesPtr inlier_pts(new pcl::Indices);
        for (auto &p: pc->points) {
            kdtree->radiusSearch(p, nn_inlier_threshold, k_indices, k_sqr_distances,
                                 1);
            if (!k_indices.empty()) {
                inlier_pts->push_back(k_indices[0]);
            }
        }
        oc_visible_inlier_pt_idxs.push_back(inlier_pts);
    }
    std::cout << "Inliers and visiblity init elapsed time: " << chronometer.toc() << "s," << "Render Time: "
              << render_time << "s\n";
}

void GeneticEvaluatorOC::init_collisions() {
    collisions.pairs.clear();
    // Detect collisions
    chronometer.tic();
    collisions = get_collisions(dp.ocs, meshPtr);
    std::cout << "Collision init elapsed time: " << chronometer.toc() << "s\n";
}

double GeneticEvaluatorOC::evaluate_chromosome(chromosomeT &chromosome) {

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
    std::vector<bool> in_collision(chromosome.size(), false);
    std::vector<double> max_pen_dist(chromosome.size(), 0);
    for (int i = 0; i < collisions.pairs.size(); i++) {
        auto &cp = collisions.pairs[i];
        double &pen_dist = collisions.distances[i];
        if (chromosome[cp.first] && chromosome[cp.second]) {
            in_collision[cp.first] = true;
            in_collision[cp.second] = true;

            max_pen_dist[cp.first] = std::max(max_pen_dist[cp.first], pen_dist);
            max_pen_dist[cp.second] = std::max(max_pen_dist[cp.second], pen_dist);
        }
    }


    for (int i = 0; i < chromosome.size(); i++) {
        if (chromosome[i]) {
            int vis_inlier_pt_cnt = oc_visible_inlier_pt_idxs[i]->size();
            vis_pt_cnt_tot += visible_oc_pcs[i]->points.size();

            if (!in_collision[i]) {
                vis_inlier_pt_cnt_tot += vis_inlier_pt_cnt;
            } else {
                vis_inlier_pt_cnt_tot += sigmoid_fall_off(max_pen_dist[i]) * vis_inlier_pt_cnt;
            }
            n_active_genes++;
        }
    }
  // oc_inlier_threshold determins the fraction of vis_pt_cnt_tot/oc_inlier_threshold that creats zero extra cost. If this fraction is higher than oc_inlier_threshold cost decreases
    cost += (vis_pt_cnt_tot - (1.0 / oc_inlier_threshold) * vis_inlier_pt_cnt_tot);

    if (isnan(cost))
        cost = std::numeric_limits<double>::max();

    return cost;
}

double GeneticEvaluatorOC::evaluate_chromosome(chromosomeT &chromosome, std::vector<double> coefficients) {
    return 0;
}

double GeneticEvaluatorOC::sigmoid_fall_off(double x) {
    return 1 /
           (1 + std::exp(-sigmoid_falloff_scale * (x - sigmoid_falloff_center)));
}

void GeneticEvaluatorOC::getHyperParameters_d(std::vector<std::string> &names, std::vector<double *> &params) {
    params.emplace_back(&nn_inlier_threshold);
    params.emplace_back(&sigmoid_falloff_center);
    params.emplace_back(&sigmoid_falloff_scale);
    params.emplace_back(&oc_inlier_threshold);

    names.emplace_back("nn_inlier_threshold");
    names.emplace_back("sigmoid_falloff_center");
    names.emplace_back("sigmoid_falloff_scale");
    names.emplace_back("oc_inlier_threshold");
}

void GeneticEvaluatorOC::setHyperParameters_d(vector<double> &params) {
    nn_inlier_threshold=params[0];
    sigmoid_falloff_center=params[1];
    sigmoid_falloff_scale=params[2];
    oc_inlier_threshold=params[3];
}



