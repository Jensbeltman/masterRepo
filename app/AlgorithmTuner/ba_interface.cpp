//
// Created by jens on 1/20/21.
//

#include "ba_interface.hpp"

BAInterface::BAInterface():AlgorithmInterface()  {
    name="BA";
}

rawDataT BAInterface::run(GeneticEvaluatorOCPtr &geneticEvaluatorOcPtr) {
    chronometer.tic();
    BAResult baResult;
    Baseline baseline(geneticEvaluatorOcPtr);
    baResult = baseline.solve();
    return rawDataT{geneticEvaluatorOcPtr->dp,baResult.chromosome,chronometer.toc()};
}
