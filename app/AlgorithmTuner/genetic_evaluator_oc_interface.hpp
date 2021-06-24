
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

class GeneticEvaluatorInlierScaledInterface: public EvaluatorInterface {
public:
    GeneticEvaluatorInlierScaledInterface();
    GeneticEvaluatorInlierScaledPtr geneticEvaluatorInlierScaledPtr;
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

class GeneticEvaluatorLRInterface: public EvaluatorInterface {
public:
    GeneticEvaluatorLRInterface();
    GeneticEvaluatorLRPtr geneticEvaluatorLRPtr;
};
class GeneticEvaluatorLRCInterface: public EvaluatorInterface {
public:
    GeneticEvaluatorLRCInterface();
    GeneticEvaluatorLRCPtr geneticEvaluatorLRCPtr;
};

class GeneticEvaluatorLRSInterface: public EvaluatorInterface {
public:
    GeneticEvaluatorLRSInterface();
    GeneticEvaluatorLRSPtr geneticEvaluatorLRSPtr;
};

class GeneticEvaluatorF1Interface: public EvaluatorInterface {
public:
    GeneticEvaluatorF1Interface();
    GeneticEvaluatorF1Ptr geneticEvaluatorF1Ptr;
};
class GeneticEvaluatorPrecisionInterface: public EvaluatorInterface {
public:
    GeneticEvaluatorPrecisionInterface();
    GeneticEvaluatorPrecisionPtr geneticEvaluatorPrecisionPtr;
};



#endif //MASTER_GENETIC_EVALUATOR_OC_INTERFACE_HPP
