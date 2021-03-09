#include <hypothesis_verification/evaluator/point_cloud_renderer.hpp>
#include "hypothesis_verification/evaluator/functionality/VisibleInlier.hpp"

VisibleInlier::VisibleInlier(double nn_inlier_threshold): nn_inlier_threshold(nn_inlier_threshold) {}
void VisibleInlier::init_visible_inliers() {
    chronometer.tic();
    int n_ocs = dp.ocs.size();
    oc_visible_inlier_pt_idxs.clear();
    PointCloudRenderer pc_render;
    pc_render.addActorsPLY(datasetObjectPtr->mesh_path, dp.ocs);
    pc_render.fitCameraAndResolution();
    visible_oc_pcs.clear();
    visible_oc_pcs.reserve(n_ocs);
    pc_render.renderPointClouds(visible_oc_pcs);

    double render_time = chronometer.toc();

    // Down sample rendered point cloud to match resolution of pcm and pc
    for (auto &vpc:visible_oc_pcs) {
        voxelGridPtr->setInputCloud(vpc);
        voxelGridPtr->filter(*vpc);
    }

    // Vectors for knn
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    oc_visible_inlier_pt_idxs.resize(n_ocs);
    for (int i = 0; i < n_ocs; i++) {
        auto &pc = visible_oc_pcs[i];

        pcl::IndicesPtr &inlier_pts = oc_visible_inlier_pt_idxs[i];
        if (inlier_pts == nullptr) {
            inlier_pts = pcl::make_shared<pcl::Indices>();
            inlier_pts->reserve(pc->points.size());
        } else {
            inlier_pts->clear();
        }

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

