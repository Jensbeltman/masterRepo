#include "hv_interfaces.hpp"
#include "hypothesis_verification/hv_alg/ga_functions.hpp"

//BA
SPInterface::SPInterface(): HVInterface()  {
    name="SP";
    variables_d.emplace_back(var_d{&sequentialPrior.score_threshold,new QDoubleSpinBox,"Score Threshold",std::to_string(sequentialPrior.score_threshold)});
}

void SPInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, rawDataT &rawData) {
    rawData.dp = geneticEvaluatorPtr->dp;
    chronometer.tic();
    HVResult hvResult;
    sequentialPrior.geneticEvaluatorPtr = geneticEvaluatorPtr;
    rawData.hvResult = sequentialPrior.solve();
    rawData.time = chronometer.toc();
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

void GAInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, rawDataT &rawData) {
    chronometer.tic();
    GAResult gaResult;
    ga.n_genes = geneticEvaluatorPtr->dp.ocs.size();
    ga.geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorPtr);
    rawData.dp = geneticEvaluatorPtr->dp;
    rawData.hvResult =  ga.solve();
    rawData.time = chronometer.toc();
}

GAWInterface::GAWInterface() {
    name="GAW";
    ga.generate_initial_population = initialize_population_score_sampling;
}


BaselineInterface::BaselineInterface(): HVInterface()  {
    name="BL";
    variables_d.emplace_back(var_d{&score_threshold,new QDoubleSpinBox,"Score Threshold",std::to_string(score_threshold)});
}

void BaselineInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, rawDataT &rawData) {
    chronometer.tic();
    chromosomeT chromosome(geneticEvaluatorPtr->dp.ocs.size());
    for(int i = 0;i<chromosome.size();i++)
        chromosome[i]=geneticEvaluatorPtr->dp.oc_scores[i]>score_threshold;


    rawData.dp = geneticEvaluatorPtr->dp;
    rawData.hvResult.cost = geneticEvaluatorPtr->evaluate_chromosome(chromosome);
    rawData.hvResult.cost_history.emplace_back(rawData.hvResult.cost);
    rawData.hvResult.chromosome = chromosome;
    rawData.time = chronometer.toc();
}

GASPInterface::GASPInterface() {
    name="GASP";
    variables_d.emplace_back(var_d{&sequentialPrior.score_threshold,new QDoubleSpinBox,"SP Score Threshold",std::to_string(sequentialPrior.score_threshold)});

}

void GASPInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, rawDataT &rawData) {
    chronometer.tic();
    // Get SP solution
    HVResult spResult;
    sequentialPrior.geneticEvaluatorPtr = geneticEvaluatorPtr;
    spResult = sequentialPrior.solve();

    GAResult gaResult;
    ga.n_genes = geneticEvaluatorPtr->dp.ocs.size();
    ga.geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorPtr);

    // Initialize GA based on SP
    ga.generate_initial_population= nullptr;
    ga.population.resize(ga.population_size);
    ga.population[0] = spResult.chromosome;
    for(int i =1;i<ga.population_size;i++){
        ga.mutation(&ga,spResult.chromosome);
        ga.population[i]=spResult.chromosome;
    }

    // solve with GA
    rawData.dp = geneticEvaluatorPtr->dp;
    rawData.hvResult =  ga.solve();
    rawData.time = chronometer.toc();
}
