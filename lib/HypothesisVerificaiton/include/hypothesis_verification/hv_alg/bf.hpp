#ifndef MASTER_BF_HPP
#define MASTER_BF_HPP
#include "hypothesis_verification/hv_alg/bf_gen.hpp"
#include "hypothesis_verification/evaluator/GeneticEvaluator.hpp"
#include "hypothesis_verification/typedefinitions.hpp"
class BF {
public:
    GeneticEvaluatorPtr geneticEvaluatorPtr;
    chromosomeT run();
    chromosomeT best_chromosome;
    double best_cost;
};


#endif //MASTER_BF_HPP
