#include "../../lib/HypothesisVerificaiton/include/hypothesis_verification/hv_alg/sequential_prior.hpp"
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
    if(!std::filesystem::exists(output_folder))
        std::filesystem::create_directory(output_folder);

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
    bool vis_on = (bool)std::stoi(argv[3]);

    TransformUtility tu;

    // Logging
    rapidcsv::CSVWriteDoc ga_cost_hist_doc(outputfolder + "/gaCostHist.csv",
                                           rapidcsv::LabelParams(-1, 0));
    rapidcsv::CSVWriteDoc ba_cost_hist_doc(output_folder + "/baCostHist.csv",
                                           rapidcsv::LabelParams(-1, 0));
    rapidcsv::CSVWriteDoc results_doc(output_folder+"/results.csv", rapidcsv::LabelParams(0, 0));
    results_doc.SetColumnName(0, "GA_Chromosome");
    results_doc.SetColumnName(1, "BA_Chromosome");
    results_doc.SetColumnName(2, "#ocs");
    results_doc.SetColumnName(3, "#ga_tp");
    results_doc.SetColumnName(4, "#ga_fp");
    results_doc.SetColumnName(5, "#ga_tn");
    results_doc.SetColumnName(6, "#ga_fn");
    results_doc.SetColumnName(7, "#ba_tp");
    results_doc.SetColumnName(8, "#ba_fp");
    results_doc.SetColumnName(9, "#ba_tn");
    results_doc.SetColumnName(10, "#ba_fn");

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
        GA ga(100, 20, 50, 0.05, 0.1, 0.3);
        ga.geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorOCPtr);

        SequentialPrior baseline(geneticEvaluatorOCPtr);

        for (int sample_n = 0; sample_n < ob->size(); sample_n++) {
            if (ob->has_gt(sample_n)) {
                geneticEvaluatorOCPtr->init_datapoint(sample_n);

                auto &dp = geneticEvaluatorOCPtr->dp;

                std::cout << "Sample number: " << sample_n << " pc file "
                          << ob->scape_data_points[sample_n].pcd_filename << std::endl;

                HVResult baResult = baseline.solve();

                //GA
                ga.n_genes = dp.ocs.size(); // Update chromosome size
                GAResult gaResult = ga.solve();

                std::vector<bool> correct_ocs = tu.find_correct_ocs(dp.ocs, dp.gts, 4, 5);

                int ga_tp = 0;
                int ga_fp = 0;
                int ga_tn = 0;
                int ga_fn = 0;
                int ba_tp = 0;
                int ba_fp = 0;
                int ba_tn = 0;
                int ba_fn = 0;

                int n_oc = dp.ocs.size();
                int n_fp = 0;
                for (int i = 0; i < n_oc; i++) {
                    if (correct_ocs[i]) {
                        n_fp++;
                        if (gaResult.chromosome[i]) {
                            ga_tp++;
                        }else{
                            ga_fn++;
                        }
                        if (baResult.chromosome[i]) {
                            ba_tp++;
                        }else{
                            ba_fn++;
                        }
                    }
                    else{
                        if (gaResult.chromosome[i]) {
                            ga_fp++;
                        }else{
                            ga_tn++;
                        }
                        if (baResult.chromosome[i]) {
                            ba_fp++;
                        }else{
                            ba_tn++;
                        }
                    }
                }

                std::string row_name = ob->name_singular + "_" + std::to_string(sample_n);
                ga_cost_hist_doc.AppendRowNameAndValue(csv_row, row_name, gaResult.cost_history);
                ba_cost_hist_doc.AppendRowNameAndValue(csv_row, row_name, baResult.cost_history);
                results_doc.SetCell(0,csv_row, gaResult.chromosome);
                results_doc.SetCell(1,csv_row, baResult.chromosome);
                results_doc.SetCell(2,csv_row, correct_ocs);
                results_doc.SetCell(3,csv_row, ga_tp);
                results_doc.SetCell(4,csv_row, ga_fp);
                results_doc.SetCell(5,csv_row, ga_tn);
                results_doc.SetCell(6,csv_row, ga_fn);
                results_doc.SetCell(7,csv_row, ba_tp);
                results_doc.SetCell(8,csv_row, ba_fp);
                results_doc.SetCell(9,csv_row, ba_tn);
                results_doc.SetCell(10,csv_row, ba_fn);
                results_doc.SetRowName(csv_row, row_name);
                csv_row++;

                std::cout <<dp.ocs.size()<<"ocs "<<dp.gts.size()<<"gts "<<"results(tp,fp,tn,fn), GA: " << ga_tp <<"\t"<<ga_fp<<"\t"<<ga_tn<<"\t"<<ga_fn<<" BA: "<< ba_tp <<"\t"<<ba_fp<<"\t"<<ba_tn<<"\t"<<ba_fn<<std::endl;

                if (vis_on) { //vis_on || (ga_fp<ba_fp) /todo fix visualizer
                    PointCloudGroupVisualizer vis;
                    // VISUALISATION
                    vis.addIdPointCloud(geneticEvaluatorOCPtr->pc, "Captured Point Cloud");
                    PointCloudT::Ptr mesh_pc = geneticEvaluatorOCPtr->pcm;
                    pcl::ExtractIndices<PointT> extractIndices;

                    for (int i = 0; i < geneticEvaluatorOCPtr->dp.ocs.size(); i++) {
                        std::string id = "oc_" + std::to_string(i);

                        if (gaResult.chromosome[i] && baResult.chromosome[i]) {
                            vis.addIdPointCloud(geneticEvaluatorOCPtr->visible_oc_pcs[i], id, "Accepted by both", 0, 255, 0);
                        } else if ((!gaResult.chromosome[i]) && (!baResult.chromosome[i])) {
                            vis.addIdPointCloud(geneticEvaluatorOCPtr->visible_oc_pcs[i], id, "Rejected by both", 255, 0, 0);
                        } else if ((gaResult.chromosome[i]) && (!baResult.chromosome[i])) {
                            vis.addIdPointCloud(geneticEvaluatorOCPtr->visible_oc_pcs[i], id, "Rejcted by baseline", 255, 255, 0);
                        } else if ((!gaResult.chromosome[i]) && (baResult.chromosome[i])) {
                            vis.addIdPointCloud(geneticEvaluatorOCPtr->visible_oc_pcs[i], id, "Accepted by baseline", 0, 0, 255);
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
    ga_cost_hist_doc.Save();
    ba_cost_hist_doc.Save();
    results_doc.Save();
    return 0;
}