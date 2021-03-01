#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollision.hpp"
#ifndef MASTER_GENETICEVALUATIONINLIERCOLLISIONVARIANTS_HPP
#define MASTER_GENETICEVALUATIONINLIERCOLLISIONVARIANTS_HPP


class GeneticEvaluatorInlierCollisionScaled: public GeneticEvaluatorInlierCollision{
public:
    GeneticEvaluatorInlierCollisionScaled();
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
};
typedef std::shared_ptr<GeneticEvaluatorInlierCollisionScaled> GeneticEvaluatorInlierCollisionScaledPtr;

class GeneticEvaluatorScoreCollision: public GeneticEvaluatorInlierCollision{
public:
    GeneticEvaluatorScoreCollision();
    void initialise_datapoint(DataPoint &datapoint_n) override;
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
};
typedef std::shared_ptr<GeneticEvaluatorScoreCollision> GeneticEvaluatorScoreCollisionPtr;

class GeneticEvaluatorUniqueInlierCollisionScaled: public GeneticEvaluatorInlierCollision{
public:
    GeneticEvaluatorUniqueInlierCollisionScaled();
    void initialise_datapoint(DataPoint &datapoint_n) override;
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
    std::vector<int> n_point_intersections;
};
typedef std::shared_ptr<GeneticEvaluatorUniqueInlierCollisionScaled> GeneticEvaluatorUniqueInlierCollisionScaledPtr;


#endif //MASTER_GENETICEVALUATIONINLIERCOLLISIONVARIANTS_HPP
