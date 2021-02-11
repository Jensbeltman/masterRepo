#ifndef MASTER_HV_INTERFACES_HPP
#define MASTER_HV_INTERFACES_HPP
#include "algorithm_interface.hpp"
#include "../../lib/HypothesisVerificaiton/include/hypothesis_verification/hv_alg/sequential_prior.hpp"
#include "hypothesis_verification/hv_alg/ga.hpp"

class SPInterface: public HVInterface {
public:
    SPInterface();
    SequentialPrior sequentialPrior;
    rawDataT run(GeneticEvaluatorPtr &geneticEvaluatorPtr) override;
};
typedef std::shared_ptr<SPInterface> SPInterfacePtr;


class GAInterface : public HVInterface {
public:
    GAInterface();
    GA ga;
    rawDataT run(GeneticEvaluatorPtr &geneticEvaluatorPtr) override;
};
typedef std::shared_ptr<GAInterface> GAInterfacePtr;

class GAWInterface : public GAInterface {
public:
    GAWInterface();
};
typedef std::shared_ptr<GAWInterface> GAWInterfacePtr;

class BaselineInterface : public HVInterface {
public:
    BaselineInterface();
    double score_threshold = 0;
    rawDataT run(GeneticEvaluatorPtr &geneticEvaluatorPtr) override;
};
typedef std::shared_ptr<BaselineInterface> BaselineInterfacePtr;



#endif //MASTER_HV_INTERFACES_HPP
