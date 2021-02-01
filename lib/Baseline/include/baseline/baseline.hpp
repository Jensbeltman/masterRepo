#ifndef MASTER_BASELINE_HPP
#define MASTER_BASELINE_HPP
#include "ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp"

struct BAResult {
    chromosomeT chromosome;
    double cost = std::numeric_limits<double>::max();
    std::vector<double> cost_history;
};

class Baseline {
public:
    Baseline(GeneticEvaluatorPtr geneticEvaluatorPtr);
    BAResult solve();

    GeneticEvaluatorPtr geneticEvaluatorPtr;
private:
    template<typename T>
    std::vector<size_t> sorted_idxs(std::vector<T> &vec);
    template<typename T>
    void sort_by_idx(std::vector<size_t> &ivec, std::vector<T> &vec);
};


#endif //MASTER_BASELINE_HPP
