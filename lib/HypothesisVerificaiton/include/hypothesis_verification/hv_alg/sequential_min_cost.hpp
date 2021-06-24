#ifndef MASTER_SEQUENTIAL_MIN_COST_HPP
#define MASTER_SEQUENTIAL_MIN_COST_HPP

#include "../evaluator/GeneticEvaluator.hpp"
#include "hypothesis_verification/hv_alg/hv_result.hpp"


class SequentialMinCost {

    public:
        SequentialMinCost(GeneticEvaluatorPtr geneticEvaluatorPtr = nullptr);
        HVResult solve();

        GeneticEvaluatorPtr geneticEvaluatorPtr;
        double score_threshold = 0;
};


#endif //MASTER_SEQUENTIAL_MIN_COST_HPP
