#ifndef MASTER_BA_INTERFACE_HPP
#define MASTER_BA_INTERFACE_HPP
#define MASTER_GA_WRAPPER_HPP
#include "algorithm_interface.hpp"
#include "baseline/baseline.hpp"

class BAInterface: public AlgorithmInterface {
public:
    BAInterface();
    void run(GeneticEvaluatorOCPtr &geneticEvaluatorOcPtr, std::vector<bool> &correct_ocs, std::vector<int> &tp,
             std::vector<int> &fp, std::vector<int> &tn, std::vector<int> &fn) override;
};
typedef std::shared_ptr<BAInterface> BAInterfacePtr;
#endif //MASTER_BA_INTERFACE_HPP
