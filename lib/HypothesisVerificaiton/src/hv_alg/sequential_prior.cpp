#include "../../include/hypothesis_verification/hv_alg/sequential_prior.hpp"

SequentialPrior::SequentialPrior(GeneticEvaluatorPtr geneticEvaluatorPtr): geneticEvaluatorPtr(geneticEvaluatorPtr) {

}

SPResult SequentialPrior::solve() {
    // BASELINE
    SPResult result;
    std::vector<T4> &object_candidates = geneticEvaluatorPtr->dp.ocs;
    std::vector<T4> &gts = geneticEvaluatorPtr->dp.gts;
    std::vector<double> &object_candidates_scores = geneticEvaluatorPtr->dp.oc_scores;
    std::vector<size_t> sorted_score_idx = sorted_idxs(object_candidates_scores);

    result.chromosome = chromosomeT(object_candidates.size(), false);
    result.cost = std::numeric_limits<double>::max();
    double cost;
    for (auto &i:sorted_score_idx) {
        result.chromosome[i] = true;
        cost = geneticEvaluatorPtr->evaluate_chromosome(result.chromosome);
        if (cost < result.cost) {
            result.cost = cost;
        } else {
            result.chromosome[i] = false;
        }

        result.cost_history.emplace_back(result.cost);
    }
    return result;
}

template<typename T>
std::vector<size_t> SequentialPrior::sorted_idxs(std::vector<T> &vec) {
    std::vector<size_t> idxs(vec.size());
    std::iota(idxs.begin(), idxs.end(), 0);
    std::stable_sort(idxs.begin(), idxs.end(), [&vec](size_t i1, size_t i2) { return vec[i1] > vec[i2]; });
    return idxs;
}

template<typename T>
void SequentialPrior::sort_by_idx(std::vector<size_t> &ivec, std::vector<T> &vec) {
    std::vector<T> temp = vec;
    for (auto &i:ivec)
        vec[i] = temp[i];
}