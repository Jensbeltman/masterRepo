#include <ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp>
#include <ga/ga_functions.hpp>
#include <ga/visualization/point_cloud_group_visualizer.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <dataset/scape/ScapeDataset.hpp>
#include <chronometer.h>
#include <utility>
#include <numeric>
#include <algorithm>
#include "../../lib/Baseline/include/baseline/baseline_comp.hpp"


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

int main(int argc, char** argv) {
    int sample_n = std::stoi(argv[1]);
    std::string object_name = argv[2];
    Chronometer chronometer;
    ScapeDataset scapeData("/home/jens/masterData/ScapeDataset/Scape/Full Dataset",
                           "/home/jens/masterData/ScapeDataset/Data from Scape Recognition");


    ScapeDatasetObjectPtr ob = std::dynamic_pointer_cast<ScapeDatasetObject>(scapeData.get_object_by_name(object_name));

    auto geneticEvaluatorOCPtr = std::make_shared<GeneticEvaluatorOC>(ob, sample_n,1);
    std::vector<T4> object_candidates = ob->get_object_candidates(sample_n);
    std::vector<double> object_candidates_scores = ob->get_scores(sample_n);
    std::vector<size_t> sorted_score_idx = sorted_idxs(object_candidates_scores);

    chromosomeT chromosome(object_candidates.size(),false);
    double best_cost = std::numeric_limits<double>::max();
    double cost;
    for (auto &i:sorted_score_idx) {
        chromosome[i]=true;
        cost =  geneticEvaluatorOCPtr->evaluate_chromosome(chromosome);
        if(cost<best_cost){
            best_cost=cost;
        }
        else{
            chromosome[i]=false;
        }
    }

    GA ga(object_candidates.size());
    ga.geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorOCPtr);
    ga.result.best_chromosome=chromosome;

    PointCloudGroupVisualizer vis;
    pcl::ExtractIndices<PointT> extractIndices;
    vis.addIdPointCloud(geneticEvaluatorOCPtr->pc, "Captured Point Cloud");

    for (int i = 0; i < geneticEvaluatorOCPtr->object_candidates.size(); i++) {
        std::string id = "oc_" + std::to_string(i);
        PointCloudT::Ptr ocpc(new PointCloudT);
        extractIndices.setInputCloud(geneticEvaluatorOCPtr->pcm);
        extractIndices.setIndices(geneticEvaluatorOCPtr->oc_visible_pt_idxs[i]);
        extractIndices.filter(*ocpc);
        pcl::transformPointCloud(*ocpc, *ocpc, geneticEvaluatorOCPtr->object_candidates[i]);

        if (ga.result.best_chromosome[i]) {
            vis.addIdPointCloud(ocpc, id, "Accepted", 0, 255, 0);
        } else{
            vis.addIdPointCloud(ocpc, id, "Rejected", 255, 0, 0);
        }
    }
    vis.custom_spin();
}
