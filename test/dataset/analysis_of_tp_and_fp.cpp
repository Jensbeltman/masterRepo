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
    csvDoc.SetColumnName(6, "visibleInliers");
    csvDoc.SetColumnName(7, "inCollisionInternal");
    csvDoc.SetColumnName(8, "inCollisionExternal");
    csvDoc.SetColumnName(9, "penetrationInternal");
    csvDoc.SetColumnName(10, "penetrationExternal");
    csvDoc.SetColumnName(11, "intersectingInliersInternal");
    csvDoc.SetColumnName(12, "intersectingInliersExternal");
    csvDoc.SetColumnName(13, "objectPoints");


    int ndps = 0;
    for (auto &obj:scapeDataset->objects)
        for (auto &dp:obj->data_points)
            if(dp.gts.size()>1)
                ndps+=dp.ocs.size();

    int current_dp = 0;
    int rowIdx = 0;


    GeneticEvaluatorUniqueInlierCollisionScaled ge;
    ge.vg_leaf_size = 3;
    ge.nn_inlier_threshold = 2.5;
    double t_thresh = 5;
    double r_thresh = 5;

    for (auto &obj:scapeDataset->objects) {
        ge.init(obj);
        auto &dps = obj->data_points;
        for (int dpi = 0; dpi < dps.size(); dpi++) {
            auto &dp = dps[dpi];
            tu::non_maximum_supression(dp,t_thresh, r_thresh,obj->symmetry_transforms);
            if(dp.gts.size()>1){

                ge.init_datapoint(dp);
                std::vector<int> correct_oc_indices;
                tu::find_correct_ocs(dp.ocs, dp.gts, t_thresh, r_thresh, correct_oc_indices, obj->symmetry_transforms);
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
                        penetration_external[cp.first] = std::max(ge.collisions.distances[i],penetration_external[cp.first]);
                        penetration_external[cp.second] = std::max(ge.collisions.distances[i],penetration_external[cp.second]);
                        in_collision_external[cp.first] = true;
                        in_collision_external[cp.second] = true;

                    }else{
                        penetration_internal[cp.first] = std::max(ge.collisions.distances[i],penetration_internal[cp.first]);
                        penetration_internal[cp.second] = std::max(ge.collisions.distances[i],penetration_internal[cp.second]);
                        in_collision_internal[cp.first] = true;
                        in_collision_internal[cp.second] = true;
                    }
                }
                int pair_idx=0;
                for(int i = 0; i<(ideal_chromosome.size()-1);i++) {
                    for (int j = i + 1; j < ideal_chromosome.size(); j++) {
                        if (ideal_chromosome[i] ^ ideal_chromosome[j]) {
                            max_point_intersections_external[i] = std::max(ge.collision_point_intersections[pair_idx],max_point_intersections_external[i]);
                            max_point_intersections_external[j] = std::max(ge.collision_point_intersections[pair_idx],max_point_intersections_external[j]);


                        }else{
                            max_point_intersections_internal[i] = std::max(ge.collision_point_intersections[pair_idx],max_point_intersections_internal[i]);
                            max_point_intersections_internal[j] = std::max(ge.collision_point_intersections[pair_idx],max_point_intersections_internal[j]);
                        }
                        pair_idx++;
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
                    csvDoc.SetCell(csvDoc.GetColumnIdx("visibleInliers"), rowIdx,
                                   static_cast<int>(ge.oc_visible_inlier_pt_idxs[i]->size()));

                    csvDoc.SetCell(csvDoc.GetColumnIdx("inCollisionInternal"), rowIdx,static_cast<int>(in_collision_internal[i]));
                    csvDoc.SetCell(csvDoc.GetColumnIdx("inCollisionExternal"), rowIdx,static_cast<int>(in_collision_external[i]));
                    csvDoc.SetCell(csvDoc.GetColumnIdx("penetrationInternal"), rowIdx,penetration_external[i]);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("penetrationExternal"), rowIdx,penetration_internal[i]);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("intersectingInliersInternal"), rowIdx,max_point_intersections_internal[i]);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("intersectingInliersExternal"), rowIdx,max_point_intersections_external[i]);
                    csvDoc.SetCell(csvDoc.GetColumnIdx("objectPoints"), rowIdx,ge.pcm->size());

                    std::cout << "\r" << current_dp++ << "/" << ndps << " - " << rowIdx << std::flush;
                }
            }
        }
    }


    csvDoc.Save("/home/jens/masterRepo/test/dataset/analysis_of_tp_and_fp_"+std::to_string(t_thresh)+"_"+std::to_string(r_thresh)+"_test.csv");
}