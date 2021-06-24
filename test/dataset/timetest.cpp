#include "iostream"
#include "hypothesis_verification/evaluator/collision_checking.hpp"
#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollisionVariants.hpp"
#include "hypothesis_verification/hv_alg/ga.hpp"
#include "hypothesis_verification/hv_alg/sequential_prior.hpp"
#include "hypothesis_verification/hv_alg/sequential_min_cost.hpp"
#include "dataset/scape/ScapeDataset.hpp"
#include "dataset/sileane/SileaneDataset.hpp"
#include "datautil/csv_doc.hpp"
#include "datautil/ga_conversions.hpp"

int main() {
    Chronometer chronometer;

    ScapeDatasetPtr scapeDataset = std::make_shared<ScapeDataset>(
            "/home/jens/masterData/ScapeDatasetNew/Scape/Full_Dataset",
            "/home/jens/masterData/ScapeDatasetNew/Data from Scape Recognition");

    rapidcsv::CSVDoc csvDoc;
    csvDoc.SetColumnName(0, "objName");
    csvDoc.SetColumnName(1, "global_info");
    csvDoc.SetColumnName(2, "dpI");
    csvDoc.SetColumnName(3, "h");
    csvDoc.SetColumnName(4, "visibleHypothesisPoints");
    csvDoc.SetColumnName(5, "init_time");
    int rowIdx = 0;

    GeneticEvaluatorUniqueInlierCollisionScaledPtr ge = std::make_shared<GeneticEvaluatorUniqueInlierCollisionScaled>();
    ge->vg_leaf_size = 3;
    ge->nn_inlier_threshold = 2.5;
    ge->inlier_overlap_penalty_factor = 50;
    ge->sigmoid_center = 5;
    ge->sigmoid_growth_rate = 2;

    double t_thresh = 5;
    double r_thresh = 5;
    double col_time;
    double ii_time;
    std::string col_s = "collision";
    std::string ii_s =  "intersecting_inliers";

    for (auto &obj:scapeDataset->objects) {
            ge->init(obj);
        auto &dps = obj->data_points;
        for (int dpi = 0; dpi < dps.size(); dpi++) {
            auto &dp = dps[dpi];
            tu::non_maximum_supression(dp,t_thresh, r_thresh,obj->symmetry_transforms);
            if(dp.gts.size()>1){
                ge->init_datapoint(dp);
                ge->init_visible_inliers();

                chronometer.tic();
                ge->init_collisions();
                col_time = chronometer.toc();

                chronometer.tic();
                ge->init_intersecting_points();
                ii_time = chronometer.toc();

                int visibleHypothesisPoints = 0;
                for(auto &pc:ge->visible_oc_pcs) {
                    visibleHypothesisPoints+=pc->points.size();
                }

                csvDoc.SetCell(csvDoc.GetColumnIdx("objName"), rowIdx, obj->name);
                csvDoc.SetCell(csvDoc.GetColumnIdx("global_info"), rowIdx, col_s);
                csvDoc.SetCell(csvDoc.GetColumnIdx("dpI"), rowIdx, dpi);
                csvDoc.SetCell(csvDoc.GetColumnIdx("h"), rowIdx, static_cast<int>(dp.ocs.size()));
                csvDoc.SetCell(csvDoc.GetColumnIdx("visibleHypothesisPoints"), rowIdx,visibleHypothesisPoints);
                csvDoc.SetCell(csvDoc.GetColumnIdx("init_time"), rowIdx, col_time);

                rowIdx++;

                csvDoc.SetCell(csvDoc.GetColumnIdx("objName"), rowIdx, obj->name);
                csvDoc.SetCell(csvDoc.GetColumnIdx("global_info"), rowIdx, ii_s);
                csvDoc.SetCell(csvDoc.GetColumnIdx("dpI"), rowIdx, dpi);
                csvDoc.SetCell(csvDoc.GetColumnIdx("h"), rowIdx, static_cast<int>(dp.ocs.size()));
                csvDoc.SetCell(csvDoc.GetColumnIdx("visibleHypothesisPoints"), rowIdx,visibleHypothesisPoints);
                csvDoc.SetCell(csvDoc.GetColumnIdx("init_time"), rowIdx, ii_time);

                rowIdx++;
                }
            }
        }
    csvDoc.Save("/home/jens/masterRepo/test/dataset/init_time_new"+std::to_string(t_thresh)+"_"+std::to_string(r_thresh)+".csv");
}