#ifndef MASTER_GENETICEVALUATIONINLIERCOLLISIONVARIANTS_HPP
#define MASTER_GENETICEVALUATIONINLIERCOLLISIONVARIANTS_HPP

#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollision.hpp"
#include "hypothesis_verification/evaluator/functionality/VisibleInlier.hpp"
#include "hypothesis_verification/evaluator/functionality/Collision.hpp"
#include "hypothesis_verification/evaluator/functionality/IntersectingPoints.hpp"

class GeneticEvaluatorInlierCollisionScaled: public GeneticEvaluatorInlierCollision{
public:
    GeneticEvaluatorInlierCollisionScaled();
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
};
typedef std::shared_ptr<GeneticEvaluatorInlierCollisionScaled> GeneticEvaluatorInlierCollisionScaledPtr;

class GeneticEvaluatorScoreCollision: public GeneticEvaluatorInlierCollision{
public:
    GeneticEvaluatorScoreCollision();
    void init_datapoint(DataPoint &datapoint_n) override;
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
};
typedef std::shared_ptr<GeneticEvaluatorScoreCollision> GeneticEvaluatorScoreCollisionPtr;

class GeneticEvaluatorUniqueInlierCollisionScaled: public IntersectingPoints{
public:
    GeneticEvaluatorUniqueInlierCollisionScaled();
    void init_datapoint(DataPoint &datapoint) override;
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
    double oc_inlier_threshold;
    double inlier_overlap_penalty_factor=2.0;
};
typedef std::shared_ptr<GeneticEvaluatorUniqueInlierCollisionScaled> GeneticEvaluatorUniqueInlierCollisionScaledPtr;

class GeneticEvaluatorLR: public IntersectingPoints{
public:
    GeneticEvaluatorLR();
    void init_datapoint(DataPoint &datapoint) override;
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
    double score_w;
    double visiblePointsFrac_w;
    double visibleInlierFrac_w;
    double penetration_w;
    double intersectingInliersFrac_w;
    double intercept;
};
typedef std::shared_ptr<GeneticEvaluatorLR> GeneticEvaluatorLRPtr;

class GeneticEvaluatorF1: public GeneticEvaluatorInlierCollision{
public:
    GeneticEvaluatorF1();
    double t_thresh, r_thresh;
    int tp,tn,fp,fn;
    void init_datapoint(DataPoint &datapoint) override;
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
