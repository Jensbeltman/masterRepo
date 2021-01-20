//
// Created by jens on 1/20/21.
//

#include "ba_interface.hpp"

BAInterface::BAInterface():AlgorithmInterface()  {
    name="BA";
}

void BAInterface::run(GeneticEvaluatorOCPtr &geneticEvaluatorOcPtr, std::vector<bool> &correct_ocs, std::vector<int> &tp,
                      std::vector<int> &fp, std::vector<int> &tn, std::vector<int> &fn) {
    BAResult baResult;
    Baseline baseline(geneticEvaluatorOcPtr);
    baResult = baseline.solve();
    getFPTN(tp, fp, tn, fn, baResult.chromosome, correct_ocs);
}
