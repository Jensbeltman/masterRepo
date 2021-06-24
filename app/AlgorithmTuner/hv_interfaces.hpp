#ifndef MASTER_HV_INTERFACES_HPP
#define MASTER_HV_INTERFACES_HPP
#include "algorithm_interface.hpp"
#include "../../lib/HypothesisVerificaiton/include/hypothesis_verification/hv_alg/sequential_prior.hpp"
#include "../../lib/HypothesisVerificaiton/include/hypothesis_verification/hv_alg/sequential_min_cost.hpp"
#include "hypothesis_verification/hv_alg/ga.hpp"
#include "hypothesis_verification/hv_alg/bf.hpp"

class BFInterface: public HVInterface {
public:
    BFInterface();
    BF bf;
    void run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) override;
};
typedef std::shared_ptr<BFInterface> BFInterfacePtr;

class SPInterface: public HVInterface {
public:
    SPInterface();
    SequentialPrior sequentialPrior;
    void run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) override;
};
typedef std::shared_ptr<SPInterface> SPInterfacePtr;

class BLSPInterface: public SPInterface {
public:
    BLSPInterface();
    void run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) override;
};
typedef std::shared_ptr<BLSPInterface> BLSPInterfacePtr;

class SMCInterface: public HVInterface {
public:
    SMCInterface();
    SequentialMinCost sequentialMinCost;
    void run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) override;
};
typedef std::shared_ptr<SMCInterface> SMCInterfacePtr;

class BLSMCInterface: public SMCInterface {
public:
    BLSMCInterface();
    void run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) override;
};
typedef std::shared_ptr<BLSMCInterface> BLSMCInterfacePtr;


class GAInterface : public HVInterface {
public:
    GAInterface();
    GA ga;
    bool group_optim = false;
    void run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) override;
};
typedef std::shared_ptr<GAInterface> GAInterfacePtr;

class GAWInterface : public GAInterface {
public:
    GAWInterface();
};
typedef std::shared_ptr<GAWInterface> GAWInterfacePtr;

class GASPInterface : public GAInterface{
public:
    GASPInterface();
    SequentialPrior sequentialPrior;
    double score_threshold = 0;
    GA ga;
    void run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) override;
};
typedef std::shared_ptr<GASPInterface> GASPInterfacePtr;

class BaselineInterface : public HVInterface {
public:
    BaselineInterface();
    double score_threshold = 0;
    void run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) override;
};
typedef std::shared_ptr<BaselineInterface> BaselineInterfacePtr;

class BLGAInterface : public GAInterface{
public:
    BLGAInterface();
    double score_threshold = 0;;
    void run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) override;
};
typedef std::shared_ptr<BLGAInterface> BLGAInterfacePtr;

class RandomInterface : public HVInterface {
public:
    RandomInterface();
    // random generators and distributions
    std::mt19937_64 rng; // random generator
    std::bernoulli_distribution bernoulli_dist;

    void run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) override;
};
typedef std::shared_ptr<RandomInterface> RandomInterfacePtr;

class LogisticRegressionInterface : public HVInterface {
public:
    LogisticRegressionInterface();
    static double sigmoid(double x);
    double score_w;
    double visiblePointsFrac_w;
    double visibleInlierFrac_w;
    double penetration_w;
    double intersectingInliersFrac_w;
    double intercept;
    double sigmoid_cutoff = 0.5;
    void run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) override;
};
typedef std::shared_ptr<LogisticRegressionInterface> LogisticRegressionInterfacePtr;

#endif //MASTER_HV_INTERFACES_HPP
