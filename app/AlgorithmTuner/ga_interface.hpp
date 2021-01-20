//
// Created by jens on 1/20/21.
//

#ifndef MASTER_GA_INTERFACE_HPP
#define MASTER_GA_INTERFACE_HPP

#include "algorithm_interface.hpp"
#include "ga/ga.hpp"

class GAInterface : public AlgorithmInterface {
public:
    GAInterface();
    GA ga;
    void run(GeneticEvaluatorOCPtr &geneticEvaluatorOcPtr, std::vector<bool> &correct_ocs, std::vector<int> &tp,
             std::vector<int> &fp, std::vector<int> &tn, std::vector<int> &fn) override;
};

typedef std::shared_ptr<GAInterface> GAInterfacePtr;
#endif //MASTER_GA_INTERFACE_HPP
