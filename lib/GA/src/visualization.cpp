#include "ga/utility/visualization.hpp"
#include <ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp>
#include <ga/typedefinitions.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

GAResultVis::GAResultVis(GA *ga) : ga(ga) {
    vis.registerKeyboardCallback(&GAResultVis::keyboardCallback, *this);
}

void GAResultVis::vis_result() {
    // Solution visualization
    std::shared_ptr<GeneticEvaluatorOC> geneticEvaluatorOCPtr = std::dynamic_pointer_cast<GeneticEvaluatorOC>(
            ga->geneticEvaluatorPtr);

    vis.removeAllPointClouds();
    vis.addPointCloud(geneticEvaluatorOCPtr->pc, pc_id);
    PointCloudT::Ptr mesh_pc = geneticEvaluatorOCPtr->pcm;

    pcl::ExtractIndices<PointT> extractIndices;
    for (int i = 0; i < geneticEvaluatorOCPtr->object_candidates.size(); i++) {
        std::string id = "oc_" + std::to_string(i);
        PointCloudT::Ptr ocpc(new PointCloudT);
        extractIndices.setInputCloud(geneticEvaluatorOCPtr->pcm);
        extractIndices.setIndices(geneticEvaluatorOCPtr->oc_visible_pt_idxs[i]);
        extractIndices.filter(*ocpc);
        pcl::transformPointCloud(*ocpc, *ocpc, geneticEvaluatorOCPtr->object_candidates[i]);


        int rgb[3] = {0, 0, 0};
        if (ga->result.best_chromosome[i]) {
            rgb[1] = 255;
            accepted_ids.emplace_back(id);
        } else {
            rgb[0] = 255;
            rejected_ids.emplace_back(id);
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(ocpc, rgb[0], rgb[1], rgb[2]);
        vis.addPointCloud<pcl::PointXYZ>(ocpc, color, id);

        // Todo add camera

    }

    vis.spin();
}

void GAResultVis::flip_opacity(std::vector<std::string> &ids) {
    if (!ids.empty()) {
        if (vis.contains(ids[0])){
            double current_opacity;
            vis.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity, ids[0]);
            current_opacity = (current_opacity < 1.0) ? 1.0 : 0.0;
            for (auto &id:ids) {
                if (vis.contains(id)) {
                    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity,
                                                         id);
                }
            }
        }
    }
}

void GAResultVis::flip_opacity(std::string id) {
    if (vis.contains(id)) {
        double current_opacity;
        vis.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity, id);
        current_opacity = (current_opacity < 1.0) ? 1.0 : 0.0;
        vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity, id);
    }
}

void GAResultVis::keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    if (event.keyDown()) {
        std::string key = event.getKeySym();
        if (key == "p" && event.isCtrlPressed()) {
            flip_opacity(pc_id);
        }
        if (key == "o" && event.isCtrlPressed()) {
            flip_opacity(accepted_ids);
        }
        if (key == "O" && event.isCtrlPressed() && event.isShiftPressed()) {
            flip_opacity(rejected_ids);
        }
    }
}
