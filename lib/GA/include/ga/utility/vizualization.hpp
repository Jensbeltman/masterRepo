#ifndef MASTER_VIZUALIZATION_HPP
#define MASTER_VIZUALIZATION_HPP

#include <ga/ga.hpp>
#include <ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp>
#include <ga/typedefinitions.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>



void result_vis(GA *ga, DatasetObjectPtr datasetObjectPtr) {
    // Solution visualization
    std::shared_ptr<GeneticEvaluatorOC> geneticEvaluatorOCPtr = std::dynamic_pointer_cast<GeneticEvaluatorOC>(
            ga->geneticEvaluatorPtr);

    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud(geneticEvaluatorOCPtr->pc, "data");
    PointCloudT::Ptr mesh_pc = geneticEvaluatorOCPtr->pcm;

    pcl::ExtractIndices<PointT> extractIndices;
    for (int i = 0; i < geneticEvaluatorOCPtr->object_candidates.size(); i++) {
        if (ga->result.best_chromosome[i]) {
            PointCloudT::Ptr ocpc(new PointCloudT);
            extractIndices.setInputCloud(geneticEvaluatorOCPtr->pcm);
            extractIndices.setIndices(geneticEvaluatorOCPtr->oc_visible_pt_idxs[i]);
            extractIndices.filter(*ocpc);
            pcl::transformPointCloud(*ocpc, *ocpc, geneticEvaluatorOCPtr->object_candidates[i]);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(ocpc, 0, 255, 0);
            vis.addPointCloud<pcl::PointXYZ>(ocpc, color, "oc_" + std::to_string(i));
            // Todo add camera
        }
    }

    vis.spin();
}


#endif //MASTER_VIZUALIZATION_HPP
