#ifndef MASTER_HV_INTERFACES_HPP
#define MASTER_HV_INTERFACES_HPP
#include "algorithm_interface.hpp"
#include "baseline/baseline.hpp"
#include "ga/ga.hpp"

class BAInterface: public HVInterface {
public:
    BAInterface();
    rawDataT run(GeneticEvaluatorPtr &geneticEvaluatorPtr) override;
};
typedef std::shared_ptr<BAInterface> BAInterfacePtr;


class GAInterface : public HVInterface {
public:
    GAInterface();
    GA ga;
    rawDataT run(GeneticEvaluatorPtr &geneticEvaluatorPtr) override;
};
typedef std::shared_ptr<GAInterface> GAInterfacePtr;



#endif //MASTER_HV_INTERFACES_HPP
