#include "iostream"
#include "hypothesis_verification/evaluator/collision_checking.hpp"
#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollisionVariants.hpp"
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
    csvDoc.SetColumnName(2, "gI");
    csvDoc.SetColumnName(3, "g");
    csvDoc.SetColumnName(4, "score");
    csvDoc.SetColumnName(5, "visiblePoints");
    csvDoc.SetColumnName(6, "visibleInlierPoints");
    csvDoc.SetColumnName(7, "inCollisionInternal");
    csvDoc.SetColumnName(8, "inCollisionExternal");
    csvDoc.SetColumnName(9, "penetrationInternal");
    csvDoc.SetColumnName(10, "penetrationExternal");
    csvDoc.SetColumnName(11, "intersectingPointsInternal");
    csvDoc.SetColumnName(12, "intersectingPointsExternal");

    int ndps = 0;
    for (auto &obj:scapeDataset->objects)
        for (auto &dp:obj->data_points)
            ndps+=dp.ocs.size();

    int current_dp = 0;
    int rowIdx = 0;


    GeneticEvaluatorUniqueInlierCollisionScaled ge;
    ge.vg_leaf_size = 3;
    ge.nn_inlier_threshold = 2.5;

    for (auto &obj:scapeDataset->objects) {
        ge.initialise_object(obj);
        auto &dps = obj->data_points;
        for (int dpi = 0; dpi < dps.size(); dpi++) {
            auto &dp = dps[dpi];
            if(dp.gts.size()>1){
                ge.initialise_datapoint(dp);
                std::vector<int> correct_oc_indices;
                tu::find_correct_ocs(dp.ocs, dp.gts, 5, 5, correct_oc_indices, obj->symmetry_transforms);
                std::vector<bool> ideal_chromosome(dp.ocs.size(), false);
                for (auto &i:correct_oc_indices)
                    ideal_chromosome[i] = true;

                // Check if ocs are in collision
                std::vector<bool> in_collision_internal(ideal_chromosome.size(), false);
                std::vector<bool> in_collision_external(ideal_chromosome.size(), false);
                std::vector<double> penetration_external(ideal_chromosome.size(), 0);
                std::vector<double> penetration_internal(ideal_chromosome.size(), 0);
                std::vector<int> max_point_intersections_internal(ideal_chromosome.size(), 0);
                std::vector<int> max_point_intersections_external(ideal_chromosome.size(), 0);
                for (int i = 0; i < ge.collisions.pairs.size(); i++) {
                    auto &cp = ge.collisions.pairs[i];
                    if (ideal_chromosome[cp.first] ^ ideal_chromosome[cp.second]) {
                        max_point_intersections_external[cp.first] = std::max(ge.collision_point_intersections[i],max_point_intersections_external[cp.first]);
                        max_point_intersections_external[cp.second] = std::max(ge.collision_point_intersections[i],max_point_intersections_external[cp.second]);
                        penetration_external[cp.first] = std::max(ge.collisions.distances[i],penetration_external[cp.first]);
                        penetration_external[cp.second] = std::max(ge.collisions.distances[i],penetration_external[cp.second]);
                        in_collision_external[cp.first] = true;
                        in_collision_external[cp.second] = true;

                    }else{
                        max_point_intersections_internal[cp.first] = std::max(ge.collision_point_intersections[i],max_point_intersections_internal[cp.first]);
                        max_point_intersections_internal[cp.second] = std::max(ge.collision_point_intersections[i],max_point_intersections_internal[cp.second]);
                        penetration_internal[cp.first] = std::max(ge.collisions.distances[i],penetration_internal[cp.first]);
                        penetration_internal[cp.second] = std::max(ge.collisions.distances[i],penetration_internal[cp.second]);
                        in_collision_internal[cp.first] = true;
                        in_collision_internal[cp.second] = true;
                    }
                }

                for (int i = 0; i < ideal_chromosome.size(); i++, rowIdx++) {
                    csvDoc.SetCell(csvDoc.GetColumnIdx("objName"), rowIdx, obj->name);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("dpI"), rowIdx, dpi);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("gI"), rowIdx, i);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("g"), rowIdx, static_cast<int>(ideal_chromosome[i]));
                    csvDoc.SetCell(csvDoc.GetColumnIdx("score"), rowIdx,dp.oc_scores[i]);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("visiblePoints"), rowIdx,
                                   static_cast<int>(ge.visible_oc_pcs[i]->size()));
                    csvDoc.SetCell(csvDoc.GetColumnIdx("visibleInlierPoints"), rowIdx,
                                   static_cast<int>(ge.oc_visible_inlier_pt_idxs[i]->size()));

                    csvDoc.SetCell(csvDoc.GetColumnIdx("inCollisionInternal"), rowIdx,static_cast<int>(in_collision_internal[i]));
                    csvDoc.SetCell(csvDoc.GetColumnIdx("inCollisionExternal"), rowIdx,static_cast<int>(in_collision_external[i]));
                    csvDoc.SetCell(csvDoc.GetColumnIdx("penetrationInternal"), rowIdx,penetration_external[i]);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("penetrationExternal"), rowIdx,penetration_internal[i]);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("intersectingPointsInternal"), rowIdx,max_point_intersections_internal[i]);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("intersectingPointsExternal"), rowIdx,max_point_intersections_external[i]);

                    std::cout << "\r" << current_dp++ << "/" << ndps << " - " << rowIdx << std::flush;
                }
            }
        }
    }


    csvDoc.Save("/home/jens/masterRepo/test/dataset/analysis_of_tp_and_fp.csv");
}