#include <baseline/baseline.hpp>
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
#include "datautil/csv_doc.hpp"
#include "datautil/ga_conversions.hpp"

int main(int argc, char **argv) {

    std::string s = argv[1];
    std::string delimiter = ":";
    std::string output_folder = argv[2];

    size_t pos = 0;
    std::string token;
    std::vector<std::string> object_names;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        std::cout << token << std::endl;
        object_names.push_back(token);
        s.erase(0, pos + delimiter.length());
    }
    object_names.push_back(s); // Get last name


    TransformUtility tu;

    // Logging
    rapidcsv::CSVWriteDoc cost_hist_doc(output_folder+"/gaCostHist.csv",
                                        rapidcsv::LabelParams(-1, 0));
    rapidcsv::CSVWriteDoc results_doc(output_folder+"/results.csv", rapidcsv::LabelParams(0, 0));
    results_doc.SetColumnName(0, "GA_Chromosome");
    results_doc.SetColumnName(1, "BA_Chromosome");
    results_doc.SetColumnName(2, "False_Posetives");
    int csv_row = 0;

    for(auto name:object_names) {

        Chronometer chronometer;
        ScapeDataset scapeData("/home/jens/masterData/ScapeDataset/Scape/Full_Dataset",
                               "/home/jens/masterData/ScapeDataset/Data from Scape Recognition");

        ScapeDatasetObjectPtr ob = std::dynamic_pointer_cast<ScapeDatasetObject>(
                scapeData.get_object_by_name(name));
        std::cout << "Comparing baseline vs ga for object: " << name << std::endl;
        PointCloudT::Ptr gtpc = ob->get_mesh_point_cloud();


        auto geneticEvaluatorOCPtr = std::make_shared<GeneticEvaluatorOC>(ob, 0, 1);
        GA ga(100, 100, 50, 0.05, 0.1, 0.3);
        ga.geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorOCPtr);

        Baseline baseline(geneticEvaluatorOCPtr);

        for (int sample_n = 0; sample_n < ob->size(); sample_n++) {
            if (ob->has_gt(sample_n)) {
                geneticEvaluatorOCPtr->initialise_datapoint(sample_n);

                auto &dp = geneticEvaluatorOCPtr->dp;

                std::cout << "Sample number: " << sample_n << " pc file "
                          << ob->scape_data_points[sample_n].pcd_filename << std::endl;

                BAResult baResult = baseline.solve();


                //GA
                ga.n_genes = dp.ocs.size(); // Update chromosome size
                GAResult gaResult = ga.solve();

                std::vector<bool> false_positives = tu.get_false_positives(dp.ocs, dp.gts, 4, 0.5);


                int n_fp = 0;
                int bl_n_fp = 0;
                int ga_n_fp = 0;

                int n_oc = dp.ocs.size();
                for (int i = 0; i < dp.ocs.size(); i++) {
                    if (false_positives[i]) {
                        n_fp++;
                        if (gaResult.chromosome[i]) {
                            ga_n_fp++;
                        }
                        if (baResult.chromosome[i]) {
                            bl_n_fp++;
                        }
                    }
                }

                std::string row_name = ob->name_singular + "_" + std::to_string(sample_n);
                cost_hist_doc.SetRowNameAndValue(csv_row, row_name, gaResult.cost_history);
                results_doc.SetCell(0, csv_row, gaResult.chromosome);
                results_doc.SetCell(1, csv_row, baResult.chromosome);
                results_doc.SetCell(2, csv_row, false_positives);
                results_doc.SetRowName(csv_row, row_name);
                csv_row++;

                std::cout << "Number of ocs: " << n_oc << "\tNumber of false positive: " << n_fp
                          << "\tBaseline false positive: " << bl_n_fp
                          << "\tGa false positive: " << ga_n_fp << std::endl;

                if (false) { //vis_on || (ga_n_fp<bl_n_fp) /todo fix visualizer
                    PointCloudGroupVisualizer vis;
                    // VISUALISATION
                    vis.addIdPointCloud(geneticEvaluatorOCPtr->pc, "Captured Point Cloud");
                    PointCloudT::Ptr mesh_pc = geneticEvaluatorOCPtr->pcm;
                    pcl::ExtractIndices<PointT> extractIndices;

                    for (int i = 0; i < geneticEvaluatorOCPtr->dp.ocs.size(); i++) {
                        std::string id = "oc_" + std::to_string(i);
                        PointCloudT::Ptr ocpc(new PointCloudT);
                        extractIndices.setInputCloud(geneticEvaluatorOCPtr->pcm);
                        extractIndices.setIndices(geneticEvaluatorOCPtr->oc_visible_pt_idxs[i]);
                        extractIndices.filter(*ocpc);
                        pcl::transformPointCloud(*ocpc, *ocpc, geneticEvaluatorOCPtr->dp.ocs[i]);

                        if (gaResult.chromosome[i] && baResult.chromosome[i]) {
                            vis.addIdPointCloud(ocpc, id, "Accepted by both", 0, 255, 0);
                        } else if ((!gaResult.chromosome[i]) && (!baResult.chromosome[i])) {
                            vis.addIdPointCloud(ocpc, id, "Rejected by both", 255, 0, 0);
                        } else if ((gaResult.chromosome[i]) && (!baResult.chromosome[i])) {
                            vis.addIdPointCloud(ocpc, id, "Rejcted by baseline", 255, 255, 0);
                        } else if ((!gaResult.chromosome[i]) && (baResult.chromosome[i])) {
                            vis.addIdPointCloud(ocpc, id, "Accepted by baseline", 0, 0, 255);
                        }
                    }

                    // Add gt to visualization
                    for (int g = 0; g < dp.gts.size(); g++) {
                        PointCloudT::Ptr gtpc_vis = pcl::make_shared<PointCloudT>();
                        std::string id = "gt_" + std::to_string(g);
                        pcl::transformPointCloud(*gtpc, *gtpc_vis, dp.gts[g]);
                        vis.addIdPointCloud(gtpc_vis, id, "Ground Truth", 0, 255, 255);
                    }
                    vis.resetCamera();
                    std::cout << "\n";
                    vis.custom_spin();
                    std::cout << "Stopping vis" << std::endl;
                    vis.close();
                }
            }
        }
    }
    cost_hist_doc.Save();
    results_doc.Save();
    return 0;
}