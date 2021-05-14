#include "iostream"
#include "hypothesis_verification/evaluator/collision_checking.hpp"
#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollisionVariants.hpp"
#include "hypothesis_verification/hv_alg/ga.hpp"
#include "hypothesis_verification/hv_alg/sequential_prior.hpp"
#include "dataset/scape/ScapeDataset.hpp"
#include "dataset/sileane/SileaneDataset.hpp"
#include "datautil/csv_doc.hpp"
#include "datautil/ga_conversions.hpp"

int main() {
    ScapeDatasetPtr scapeDataset = std::make_shared<ScapeDataset>(
            "/home/jens/masterData/ScapeDatasetNew/Scape/Full_Dataset",
            "/home/jens/masterData/ScapeDatasetNew/Data from Scape Recognition");

    rapidcsv::CSVDoc csvDoc;
    csvDoc.SetColumnName(0, "objName");
    csvDoc.SetColumnName(1, "dpI");
    csvDoc.SetColumnName(2, "r");
    csvDoc.SetColumnName(3, "min_cost");
    csvDoc.SetColumnName(4, "min_cost_i");
    csvDoc.SetColumnName(5, "max_it");
    csvDoc.SetColumnName(6, "pop_size");

    csvDoc.SetColumnName(7, "n_ocs");


    int ndps = 0;
    for (auto &obj:scapeDataset->objects)
        for (auto &dp:obj->data_points)
            if (dp.gts.size() > 1)
                ndps++;

    int current_dp = 0;
    int rowIdx = 0;

    double t_thresh = 5;
    double r_thresh = 5;

    GeneticEvaluatorF1Ptr ge = std::make_shared<GeneticEvaluatorF1>();
    ge->t_thresh = t_thresh;
    ge->r_thresh = r_thresh;
    ge->vg_leaf_size = 3;
    ge->nn_inlier_threshold = 2.5;

    int repetitions = 10;

    for (auto &obj:scapeDataset->objects) {
        ge->init(obj);
        auto &dps = obj->data_points;
        for (int dpi = 0; dpi < dps.size(); dpi++) {
            auto &dp = dps[dpi];
            if (dp.gts.size() > 1) {
                tu::non_maximum_supression(dp, t_thresh, r_thresh, obj->symmetry_transforms);
                ge->init_datapoint(dp);
                for (int r = 0; r < repetitions; r++) {
                    for (int pop_size = 10; pop_size < 100; pop_size += 10) {
                        for (int max_it = 10; max_it < 100; max_it += 10) {
                            GA ga;
                            ga.n_genes = dp.ocs.size();
                            ga.population_size = pop_size;
                            ga.generation_max = max_it;
                            ga.geneticEvaluatorPtr = ge;
                            HVResult hvResult = ga.solve();

                            csvDoc.SetCell(csvDoc.GetColumnIdx("objName"), rowIdx, obj->name);
                            csvDoc.SetCell(csvDoc.GetColumnIdx("dpI"), rowIdx, dpi);
                            csvDoc.SetCell(csvDoc.GetColumnIdx("r"), rowIdx, r);
                            csvDoc.SetCell(csvDoc.GetColumnIdx("min_cost"), rowIdx, hvResult.cost);
                            int min_cost_i = std::distance(hvResult.cost_history.begin(),
                                                           std::find(hvResult.cost_history.begin(),
                                                                     hvResult.cost_history.end(),
                                                                     hvResult.cost));
                            csvDoc.SetCell(csvDoc.GetColumnIdx("min_cost_i"), rowIdx, min_cost_i);
                            csvDoc.SetCell(csvDoc.GetColumnIdx("max_it"), rowIdx, max_it);
                            csvDoc.SetCell(csvDoc.GetColumnIdx("pop_size"), rowIdx, pop_size);
                            csvDoc.SetCell(csvDoc.GetColumnIdx("n_ocs"), rowIdx, dp.ocs.size());

                            std::cout << "\r" << current_dp++ << " dps" << " - " << rowIdx << "  "
                                      << hvResult.cost_history.size() << std::flush;
                            rowIdx++;
                        }
                    }
                }
            }
        }
    }


    csvDoc.Save("/home/jens/masterRepo/test/ga_and_baseline/ga_f1_cost_performance_" + std::to_string(t_thresh) + "_" +
                std::to_string(r_thresh) + ".csv");
}