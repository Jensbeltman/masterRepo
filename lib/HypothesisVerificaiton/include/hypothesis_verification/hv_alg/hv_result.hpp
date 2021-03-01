#ifndef MASTER_HV_RESULT_HPP
#define MASTER_HV_RESULT_HPP
#include "hypothesis_verification/typedefinitions.hpp"

struct HVResult {
    chromosomeT chromosome;
    double cost = std::numeric_limits<double>::max();
    std::vector<double> cost_history;
    double time;
};


#endif //MASTER_HV_RESULT_HPP
