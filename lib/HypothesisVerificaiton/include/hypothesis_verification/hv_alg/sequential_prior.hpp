#ifndef MASTER_SEQUENTIAL_PRIOR_HPP
#define MASTER_SEQUENTIAL_PRIOR_HPP
#include "../evaluator/GeneticEvaluator.hpp"
#include "hypothesis_verification/hv_alg/hv_result.hpp"

class SequentialPrior {
public:
    SequentialPrior(GeneticEvaluatorPtr geneticEvaluatorPtr = nullptr);
    HVResult solve();

    GeneticEvaluatorPtr geneticEvaluatorPtr;
    double score_threshold = 0;
private:
    template<typename T>
    std::vector<size_t> sorted_idxs(std::vector<T> &vec);
    template<typename T>
    void sort_by_idx(std::vector<size_t> &ivec, std::vector<T> &vec);
};


#endif //MASTER_SEQUENTIAL_PRIOR_HPP
