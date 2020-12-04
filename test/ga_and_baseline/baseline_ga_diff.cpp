#include <ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp>
#include <ga/ga_functions.hpp>
#include <ga/visualization/point_cloud_group_visualizer.hpp>
#include <dataset/scape/ScapeDataset.hpp>
#include <dataset/transform_utility.hpp>
#include <chronometer.h>
#include <utility>
#include <numeric>
#include <algorithm>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <ga/visualization/point_cloud_group_visualizer.hpp>
#include <test_util.hpp>

int main(int argc, char **argv) {
    std::string object_name = argv[1];
    bool vis_on = static_cast<bool>(std::stoi(argv[2]));
    Chronometer chronometer;
    ScapeDataset scapeData("/home/jens/masterData/ScapeDataset/Scape/Full_Dataset",
                           "/home/jens/masterData/ScapeDataset/Data from Scape Recognition");


    ScapeDatasetObjectPtr ob = std::dynamic_pointer_cast<ScapeDatasetObject>(scapeData.get_object_by_name(object_name));
    std::cout << "Comparing baseline vs ga for object: " << object_name << std::endl;
    PointCloudT::Ptr gtpc = ob->get_mesh_point_cloud();

    PointCloudGroupVisualizer vis;
    TransformUtility tu;
    int n_oc = 0;
    int n_tp = 0;
    int bl_n_tp = 0;
    int ga_n_tp = 0;

    GA ga(100, 100, 50, 0.05, 0.1, 0.3);

    // Logging

    //auto dataObjects = json::array();

    for (int sample_n = 0; sample_n < ob->size(); sample_n++) {
        if (ob->has_gt(sample_n)) {
            std::cout << "Sample number: " << sample_n << " pc file "
                      << ob->scape_data_points[sample_n].pcd_filename << std::endl;
            auto geneticEvaluatorOCPtr = std::make_shared<GeneticEvaluatorOC>(ob, sample_n, 1);

            // BASELINE
            std::vector<T4> object_candidates = ob->get_object_candidates(sample_n);
            std::vector<T4> gts = ob->get_gt(sample_n);
            std::vector<double> object_candidates_scores = ob->get_scores(sample_n);
            std::vector<size_t> sorted_score_idx = sorted_idxs(object_candidates_scores);

            chromosomeT bl_chromosome(object_candidates.size(), false);
            double best_cost = std::numeric_limits<double>::max();
            double cost;
            for (auto &i:sorted_score_idx) {
                bl_chromosome[i] = true;
                cost = geneticEvaluatorOCPtr->evaluate_chromosome(bl_chromosome);
                if (cost < best_cost) {
                    best_cost = cost;
                } else {
                    bl_chromosome[i] = false;
                }
            }

            //GA
            ga.n_genes = object_candidates.size(); // Update chromosome size
            ga.geneticEvaluatorPtr = geneticEvaluatorOCPtr; // Create new evaluator for the given ocs
            GAResult result = ga.solve();

            std::vector<bool> true_positives = tu.get_true_positives(object_candidates, gts, 10, 6);

            n_oc += object_candidates.size();
            for (int i = 0; i < object_candidates.size(); i++) {
                if (true_positives[i]) {
                    n_tp++;
                    if (result.best_chromosome[i]) {
                        ga_n_tp++;
                    }
                    if (bl_chromosome[i]) {
                        bl_n_tp++;
                    }
                }
            }

            std::cout << "Number of ocs: " << n_oc << "\tNumber of tps: " << n_tp << "\tBaseline tps: " << bl_n_tp
                      << "\tGa tps: " << ga_n_tp << std::endl;

            if (vis_on) {
                // VISUALISATION
                vis.clear();
                vis.addIdPointCloud(geneticEvaluatorOCPtr->pc, "Captured Point Cloud");
                PointCloudT::Ptr mesh_pc = geneticEvaluatorOCPtr->pcm;
                pcl::ExtractIndices<PointT> extractIndices;

                for (int i = 0; i < geneticEvaluatorOCPtr->object_candidates.size(); i++) {
                    std::string id = "oc_" + std::to_string(i);
                    PointCloudT::Ptr ocpc(new PointCloudT);
                    extractIndices.setInputCloud(geneticEvaluatorOCPtr->pcm);
                    extractIndices.setIndices(geneticEvaluatorOCPtr->oc_visible_pt_idxs[i]);
                    extractIndices.filter(*ocpc);
                    pcl::transformPointCloud(*ocpc, *ocpc, geneticEvaluatorOCPtr->object_candidates[i]);

                    if (ga.result.best_chromosome[i] && bl_chromosome[i]) {
                        vis.addIdPointCloud(ocpc, id, "Accepted by both", 0, 255, 0);
                    } else if ((!ga.result.best_chromosome[i]) && (!bl_chromosome[i])) {
                        vis.addIdPointCloud(ocpc, id, "Rejected by both", 255, 0, 0);
                    } else if ((ga.result.best_chromosome[i]) && (!bl_chromosome[i])) {
                        vis.addIdPointCloud(ocpc, id, "Rejcted by baseline", 255, 255, 0);
                    } else if ((!ga.result.best_chromosome[i]) && (bl_chromosome[i])) {
                        vis.addIdPointCloud(ocpc, id, "Accepted by baseline", 0, 0, 255);
                    }
                }

                // Add gt to visualization
                for (int g = 0; g < gts.size(); g++) {
                    PointCloudT::Ptr gtpc_vis = pcl::make_shared<PointCloudT>();
                    std::string id = "gt_" + std::to_string(g);
                    pcl::transformPointCloud(*gtpc, *gtpc_vis, gts[g]);
                    vis.addIdPointCloud(gtpc_vis, id, "Ground Truth", 0, 255, 255);
                }
                vis.resetCamera();
                std::cout << "\n";
                vis.custom_spin();
            }

//            dataObjects.push_back(json::object(
//                    {
//                            {"dataset",  scapeData.name},
//                            {"object",   ob->name},
//                            {"sample_n", sample_n},
//                            {"ga",       result.best_chromosome},
//                            {"baseline", bl_chromosome}
//                    }
//            ));

        }
    }
//    json data_json;
//    data_json["data"] = dataObjects;

// write prettified JSON to another file
//    std::ofstream o("../../../data/baseline_ga_comp");
//    o << std::setw(4) << data_json << std::endl;
//
//    o.close();
}