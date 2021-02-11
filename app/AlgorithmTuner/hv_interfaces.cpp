#include "hv_interfaces.hpp"
#include "hypothesis_verification/hv_alg/ga_functions.hpp"

//BA
SPInterface::SPInterface(): HVInterface()  {
    name="SP";
    variables_d.emplace_back(var_d{&sequentialPrior.score_threshold,new QDoubleSpinBox,"Score Threshold",std::to_string(sequentialPrior.score_threshold)});
}

rawDataT SPInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr) {
    chronometer.tic();
    SPResult baResult;
    sequentialPrior.geneticEvaluatorPtr = geneticEvaluatorPtr;
    baResult = sequentialPrior.solve();
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

GAWInterface::GAWInterface() {
    name="GAW";
    ga.generate_initial_population = initialize_population_score_sampling;
}


BaselineInterface::BaselineInterface(): HVInterface()  {
    name="BL";
    variables_d.emplace_back(var_d{&score_threshold,new QDoubleSpinBox,"Score Threshold",std::to_string(score_threshold)});
}

rawDataT BaselineInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr) {
    chronometer.tic();
    chromosomeT chromosome(geneticEvaluatorPtr->dp.ocs.size());
    for(int i = 0;i<chromosome.size();i++)
        chromosome[i]=geneticEvaluatorPtr->dp.oc_scores[i]>score_threshold;


    return rawDataT{geneticEvaluatorPtr->dp,chromosome,chronometer.toc()};
}

