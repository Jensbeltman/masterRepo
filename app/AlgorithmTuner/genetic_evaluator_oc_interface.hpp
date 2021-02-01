
#ifndef MASTER_GENETIC_EVALUATOR_OC_INTERFACE_HPP
#define MASTER_GENETIC_EVALUATOR_OC_INTERFACE_HPP
#include "hypothesis_verification/evaluator/GeneticEvaluatorObjectCandidates.hpp"
#include "algorithm_interface.hpp"

class GeneticEvaluatorOCInterface: public EvaluatorInterface {
public:
    GeneticEvaluatorOCInterface();
    GeneticEvaluatorOCPtr geneticEvaluatorOcPtr;

};


#endif //MASTER_GENETIC_EVALUATOR_OC_INTERFACE_HPP
