
#ifndef MASTER_GENETIC_EVALUATOR_OC_INTERFACE_HPP
#define MASTER_GENETIC_EVALUATOR_OC_INTERFACE_HPP
#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollision.hpp"
#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollisionVariants.hpp"
#include "algorithm_interface.hpp"

class GeneticEvaluatorInlierCollisionInterface: public EvaluatorInterface {
public:
    GeneticEvaluatorInlierCollisionInterface();
    GeneticEvaluatorInlierCollisionPtr geneticEvaluatorOcPtr;

};

class GeneticEvaluatorInlierCollisionScaledInterface: public EvaluatorInterface {
public:
    GeneticEvaluatorInlierCollisionScaledInterface();
    GeneticEvaluatorInlierCollisionScaledPtr geneticEvaluatorInlierCollisionScaledPtr;
};

class GeneticEvaluatorScoreCollisionInterface: public EvaluatorInterface {
public:
    GeneticEvaluatorScoreCollisionInterface();
    GeneticEvaluatorScoreCollisionPtr geneticEvaluatorScoreCollisionPtr;
};

class GeneticEvaluatorUniqueInlierCollisionScaledInterface: public EvaluatorInterface {
public:
    GeneticEvaluatorUniqueInlierCollisionScaledInterface();
    GeneticEvaluatorUniqueInlierCollisionScaledPtr geneticEvaluatorUniqueInlierCollisionScaledPtr;
};

#endif //MASTER_GENETIC_EVALUATOR_OC_INTERFACE_HPP
