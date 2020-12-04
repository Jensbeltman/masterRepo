#ifndef MASTER_BASELINE_HPP
#define MASTER_BASELINE_COMP_HPP
#include "ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp"

class Baseline {
public:
    Baseline(GeneticEvaluatorOCPtr geneticEvaluatorOCPtr);
    chromosomeT get_best_chromosome();

    GeneticEvaluatorOCPtr geneticEvaluatorOCPtr;
private:
    template<typename T>
    std::vector<size_t> sorted_idxs(std::vector<T> &vec);
    template<typename T>
    void sort_by_idx(std::vector<size_t> &ivec, std::vector<T> &vec);
};


#endif //MASTER_BASELINE_HPP
