//
// Created by jens on 1/20/21.
//

#include "ba_interface.hpp"

BAInterface::BAInterface(): HVInterface()  {
    name="BA";
}

rawDataT BAInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr) {
    chronometer.tic();
    BAResult baResult;
    Baseline baseline(geneticEvaluatorPtr);
    baResult = baseline.solve();
    return rawDataT{geneticEvaluatorPtr->dp,baResult.chromosome,chronometer.toc()};
}
