#ifndef MASTER_BASELINE_HPP
#define MASTER_BASELINE_COMP_HPP
#include "ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp"

class Baseline {
    Baseline(GeneticEvaluatorOCPtr geneticEvaluatorOCPtr);
    chromosomeT get_best_chromosome();

    GeneticEvaluatorOCPtr geneticEvaluatorOCPtr;
};


#endif //MASTER_BASELINE_HPP
