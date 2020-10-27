#include <ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp>
#include <ga/ga_functions.hpp>
#include <ga/utility/visualization.hpp>
#include <dataset/scape/ScapeDataset.hpp>
#include <chronometer.h>
#include <utility>
#include <numeric>
#include <algorithm>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

// Baseline Function

template<typename T>
std::vector<size_t> sorted_idxs(std::vector<T> &vec) {
    std::vector<size_t> idxs(vec.size());
    std::iota(idxs.begin(), idxs.end(), 0);
    std::stable_sort(idxs.begin(), idxs.end(), [&vec](size_t i1, size_t i2) { return vec[i1] > vec[i2]; });
    return idxs;
}

template<typename T>
void sort_by_idx(std::vector<size_t> &ivec, std::vector<T> &vec) {
    std::vector<T> temp = vec;
    for (auto &i:ivec)
        vec[i] = temp[i];
}
// VIS function
//void flip_opacity(std::vector<std::string> &ids, pcl::visualization::PCLVisualizer &vis) {
//    if (!ids.empty()) {
//        if (vis.contains(ids[0])){
//            double current_opacity;
//            vis.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity, ids[0]);
//            current_opacity = (current_opacity < 1.0) ? 1.0 : 0.0;
//            for (auto &id:ids) {
//                if (vis.contains(id)) {
//                    vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity,
//                                                         id);
//                }
//            }
//        }
//    }
//}
//
//void flip_opacity(std::string id,pcl::visualization::PCLVisualizer &vis) {
//    if (vis.contains(id)) {
//        double current_opacity;
//        vis.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity, id);
//        current_opacity = (current_opacity < 1.0) ? 1.0 : 0.0;
//        vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, current_opacity, id);
//    }
//}
//
//const std::string pc_id = "pc";
//std::vector<std::string> common_accepted_ids;
//std::vector<std::string> common_rejected_ids;
//std::vector<std::string> baseline_rejected_ids;
//std::vector<std::string> baseline_accepted_ids;
//pcl::visualization::PCLVisualizer vis;
//
//void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
//    if (event.keyDown()) {
//        std::string key = event.getKeySym();
//        if (key == "p" && event.isCtrlPressed()) {
//            flip_opacity(pc_id);
//        }
//        if (key == "o" && event.isCtrlPressed()) {
//            flip_opacity(accepted_ids);
//        }
//        if (key == "O" && event.isCtrlPressed() && event.isShiftPressed()) {
//            flip_opacity(rejected_ids);
//        }
//    }
//}
int main(int argc, char** argv) {
    std::string object_name = argv[1];
    Chronometer chronometer;
    ScapeDataset scapeData("/home/jens/masterData/ScapeDataset/Scape/Full Dataset",
                           "/home/jens/masterData/ScapeDataset/Data from Scape Recognition");


    ScapeDatasetObjectPtr ob = std::dynamic_pointer_cast<ScapeDatasetObject>(scapeData.get_object_by_name(object_name));
    std::cout<<"Comparing baseline vs ga for object: "<<object_name<<std::endl;

    pcl::visualization::PCLVisualizer vis;
    for(int sample_n = 0;sample_n<ob->size();sample_n++) {
        std::cout<<"Sample number: "<<sample_n<<" pc file "<<ob->filenames[ob->zones[sample_n].pc_filename_idx]<<std::endl;
        auto geneticEvaluatorOCPtr = std::make_shared<GeneticEvaluatorOC>(ob, sample_n, 1);

        // BASELINE
        std::vector<T4> object_candidates = ob->get_object_candidates(sample_n);
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
        GA ga(object_candidates.size());
        ga.geneticEvaluatorPtr = geneticEvaluatorOCPtr;
        ga.N_chromosomes = 100;
        ga.generation_max = 50;
        ga.mutation_rate = 0.01;
        ga.elite_count = (int) (0.1 * ga.N_genes);
        ga.parent_pool_count = (int) (0.3 * ga.N_genes);
        ga.crossover = crossover_uniform;
        ga.mutation = mutation_flip;
        GAResult result = ga.solve();



        // VISUALISATION
        vis.removeAllPointClouds();
        vis.addPointCloud(geneticEvaluatorOCPtr->pc, "data");
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
            if (ga.result.best_chromosome[i] && bl_chromosome[i]) {
                rgb[1] = 255;
//                common_accepted_ids.emplace_back(id);
            } else if((!ga.result.best_chromosome[i]) && (!bl_chromosome[i])){
                rgb[0] = 255;
//                common_rejected_ids.emplace_back(id);
            }
            else if((ga.result.best_chromosome[i]) && (!bl_chromosome[i])){
                rgb[0] = 255;
                rgb[1] = 255;
//                baseline_rejected_ids.emplace_back(id);
            }else if((!ga.result.best_chromosome[i]) && (bl_chromosome[i])){
                rgb[2] = 255;
//                baseline_accepted_ids.emplace_back(id);
            }
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(ocpc, rgb[0], rgb[1], rgb[2]);
            vis.addPointCloud<pcl::PointXYZ>(ocpc, color, id);

            // Todo add camera

        }
        vis.resetCamera();
        std::cout<<"\n";
        vis.spin();
    }
}