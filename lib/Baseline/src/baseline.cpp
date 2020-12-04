#include "baseline/baseline.hpp"

Baseline::Baseline(GeneticEvaluatorOCPtr geneticEvaluatorOCPtr):geneticEvaluatorOCPtr(geneticEvaluatorOCPtr) {

}

chromosomeT Baseline::get_best_chromosome() {
    // BASELINE
    std::vector<T4> &object_candidates = geneticEvaluatorOCPtr->dp.ocs;
    std::vector<T4> &gts = geneticEvaluatorOCPtr->dp.gts;
    std::vector<double> &object_candidates_scores = geneticEvaluatorOCPtr->dp.oc_scores;
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
    return bl_chromosome;
}

template<typename T>
std::vector<size_t> Baseline::sorted_idxs(std::vector<T> &vec) {
    std::vector<size_t> idxs(vec.size());
    std::iota(idxs.begin(), idxs.end(), 0);
    std::stable_sort(idxs.begin(), idxs.end(), [&vec](size_t i1, size_t i2) { return vec[i1] > vec[i2]; });
    return idxs;
}

template<typename T>
void Baseline::sort_by_idx(std::vector<size_t> &ivec, std::vector<T> &vec) {
    std::vector<T> temp = vec;
    for (auto &i:ivec)
        vec[i] = temp[i];
}