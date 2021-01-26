#ifndef MASTER_BA_INTERFACE_HPP
#define MASTER_BA_INTERFACE_HPP
#define MASTER_GA_WRAPPER_HPP
#include "algorithm_interface.hpp"
#include "baseline/baseline.hpp"


class BAInterface: public AlgorithmInterface {
public:
    BAInterface();
    rawDataT run(GeneticEvaluatorOCPtr &geneticEvaluatorOcPtr) override;
};
typedef std::shared_ptr<BAInterface> BAInterfacePtr;
#endif //MASTER_BA_INTERFACE_HPP
