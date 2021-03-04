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
    void initialise_datapoint(DataPoint &datapoint) override;
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
    std::vector<int> collision_point_intersections;
};
typedef std::shared_ptr<GeneticEvaluatorUniqueInlierCollisionScaled> GeneticEvaluatorUniqueInlierCollisionScaledPtr;

class GeneticEvaluatorF1: public GeneticEvaluatorInlierCollision{
public:
    GeneticEvaluatorF1();
    double t_thresh, r_thresh;
    int tp,tn,fp,fn;
    void initialise_datapoint(DataPoint &datapoint) override;
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
};
typedef std::shared_ptr<GeneticEvaluatorF1> GeneticEvaluatorF1Ptr;

class GeneticEvaluatorPrecision: public GeneticEvaluatorF1{
public:
    GeneticEvaluatorPrecision();
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
};
typedef std::shared_ptr<GeneticEvaluatorPrecision> GeneticEvaluatorPrecisionPtr;
#endif //MASTER_GENETICEVALUATIONINLIERCOLLISIONVARIANTS_HPP
