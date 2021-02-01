#include "hv_interfaces.hpp"

//BA
SPInterface::SPInterface(): HVInterface()  {
    name="SP";
}

rawDataT SPInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr) {
    chronometer.tic();
    SPResult baResult;
    SequentialPrior baseline(geneticEvaluatorPtr);
    baResult = baseline.solve();
    return rawDataT{geneticEvaluatorPtr->dp,baResult.chromosome,chronometer.toc()};
}

// GA
GAInterface::GAInterface(): HVInterface() {
    name="GA";
    // Creating Variables
    variables_i.emplace_back(var_i{&ga.population_size,new QSpinBox,"population_size",std::to_string(ga.population_size)});
    variables_i.emplace_back(var_i{&ga.generation_max,new QSpinBox,"generation_max",std::to_string(ga.generation_max)});

    variables_d.emplace_back(var_d{&ga.mutation_rate,new QDoubleSpinBox,"mutation_rate",std::to_string(ga.mutation_rate)});
    variables_d.emplace_back(var_d{&ga.elite_pct,new QDoubleSpinBox,"elite_pct",std::to_string(ga.elite_pct)});
    variables_d.emplace_back(var_d{&ga.parent_pool_pct,new QDoubleSpinBox,"parent_pool_pct",std::to_string(ga.parent_pool_pct)});

    // Setting QT properties
    for(auto & var:variables_i)
        var.spinBox->setMaximum(10000);
}

rawDataT GAInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr) {
    chronometer.tic();
    GAResult gaResult;
    ga.n_genes = geneticEvaluatorPtr->dp.ocs.size();
    ga.geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorPtr);
    gaResult =  ga.solve();
    return rawDataT{geneticEvaluatorPtr->dp, gaResult.chromosome, chronometer.toc()};
}

BaselineInterface::BaselineInterface(): HVInterface()  {
    name="BL";
}

rawDataT BaselineInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr) {
    chronometer.tic();
    return rawDataT{geneticEvaluatorPtr->dp,chromosomeT(geneticEvaluatorPtr->dp.ocs.size(),true),chronometer.toc()};
}
