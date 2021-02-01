
#ifndef MASTER_GENETIC_EVALUATOR_OC_INTERFACE_HPP
#define MASTER_GENETIC_EVALUATOR_OC_INTERFACE_HPP
#include <ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp>
#include "algorithm_interface.hpp"

class GeneticEvaluatorOCInterface: public EvaluatorInterface {
public:
    GeneticEvaluatorOCInterface();
    GeneticEvaluatorOCPtr geneticEvaluatorOcPtr;

};


#endif //MASTER_GENETIC_EVALUATOR_OC_INTERFACE_HPP
