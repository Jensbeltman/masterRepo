#ifndef MASTER_GENETICEVALUATIONINLIERCOLLISIONVARIANTS_HPP
#define MASTER_GENETICEVALUATIONINLIERCOLLISIONVARIANTS_HPP

#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollision.hpp"
#include "hypothesis_verification/evaluator/functionality/VisibleInlier.hpp"
#include "hypothesis_verification/evaluator/functionality/Collision.hpp"
#include "hypothesis_verification/evaluator/functionality/IntersectingPoints.hpp"

class GeneticEvaluatorInlierScaled: public GeneticEvaluatorInlierCollision{
public:
    GeneticEvaluatorInlierScaled();
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
};
typedef std::shared_ptr<GeneticEvaluatorInlierScaled> GeneticEvaluatorInlierScaledPtr;

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
    double sigmoid_center;
    double sigmoid_growth_rate;
    double inlier_overlap_penalty_factor=2.0;
    double sigmoid_fall_off(double x);

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

class GeneticEvaluatorLRC: public GeneticEvaluatorLR{
public:
    GeneticEvaluatorLRC();
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
};
typedef std::shared_ptr<GeneticEvaluatorLRC> GeneticEvaluatorLRCPtr;

class GeneticEvaluatorLRS: public GeneticEvaluatorLR{
public:
    GeneticEvaluatorLRS();
    virtual double evaluate_chromosome(chromosomeT &chromosome) override;
};
typedef std::shared_ptr<GeneticEvaluatorLRS> GeneticEvaluatorLRSPtr;

class GeneticEvaluatorF1: public GeneticEvaluatorInlierCollision{
public:
    GeneticEvaluatorF1();
    double t_thresh, r_thresh;
    int tp,tn,fp,fn;
    std::vector<int> correct_oc_indices;
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
