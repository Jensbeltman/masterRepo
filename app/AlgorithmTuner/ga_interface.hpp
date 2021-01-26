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
    rawDataT run(GeneticEvaluatorOCPtr &geneticEvaluatorOcPtr) override;
};

typedef std::shared_ptr<GAInterface> GAInterfacePtr;
#endif //MASTER_GA_INTERFACE_HPP
