#include "hv_interfaces.hpp"
#include "hypothesis_verification/hv_alg/ga_functions.hpp"
#include <cmath>

//BA
SPInterface::SPInterface(): HVInterface()  {
    name="SP";
    parameters_d.emplace_back(param_d{&sequentialPrior.score_threshold, new QDoubleSpinBox, "Score Threshold", std::to_string(sequentialPrior.score_threshold)});
}

void SPInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) {
    chronometer.tic();
    sequentialPrior.geneticEvaluatorPtr = geneticEvaluatorPtr;
    hvResult = sequentialPrior.solve();
    hvResult.time = chronometer.toc();
}

// GA
GAInterface::GAInterface(): HVInterface() {
    name="GA";
    // Creating Variables
    parameters_i.emplace_back(param_i{&ga.population_size, new QSpinBox, "population_size", std::to_string(ga.population_size)});
    parameters_i.emplace_back(param_i{&ga.generation_max, new QSpinBox, "generation_max", std::to_string(ga.generation_max)});

    parameters_d.emplace_back(param_d{&ga.mutation_rate, new QDoubleSpinBox, "mutation_rate", std::to_string(ga.mutation_rate)});
    parameters_d.emplace_back(param_d{&ga.elite_pct, new QDoubleSpinBox, "elite_pct", std::to_string(ga.elite_pct)});
    parameters_d.emplace_back(param_d{&ga.parent_pool_pct, new QDoubleSpinBox, "parent_pool_pct", std::to_string(ga.parent_pool_pct)});

    // Setting QT properties
    for(auto & var:parameters_i)
        var.spinBox->setMaximum(10000);
}

void GAInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) {
    chronometer.tic();
    GAResult gaResult;
    ga.n_genes = geneticEvaluatorPtr->dp.ocs.size();
    ga.geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorPtr);
    hvResult =  ga.solve();
    hvResult.time = chronometer.toc();
}

GAWInterface::GAWInterface() {
    name="GAW";
    ga.generate_initial_population = initialize_population_score_sampling;
}


BaselineInterface::BaselineInterface(): HVInterface()  {
    name="BL";
    parameters_d.emplace_back(param_d{&score_threshold, new QDoubleSpinBox, "Score Threshold", std::to_string(score_threshold)});
}

void BaselineInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) {
    chronometer.tic();
    chromosomeT chromosome(geneticEvaluatorPtr->dp.ocs.size());
    for(int i = 0;i<chromosome.size();i++)
        chromosome[i]=geneticEvaluatorPtr->dp.oc_scores[i]>score_threshold;

    hvResult.cost = geneticEvaluatorPtr->evaluate_chromosome(chromosome);
    hvResult.cost_history.emplace_back(hvResult.cost);
    hvResult.chromosome = chromosome;
    hvResult.time = chronometer.toc();
}

GASPInterface::GASPInterface() {
    name="GASP";
    parameters_d.emplace_back(param_d{&sequentialPrior.score_threshold, new QDoubleSpinBox, "SP Score Threshold", std::to_string(sequentialPrior.score_threshold)});

}

void GASPInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) {
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
    hvResult =  ga.solve();
    hvResult.time = chronometer.toc();
}

RandomInterface::RandomInterface() {
    name="RAND";

    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
    rng.seed(ss);
}

void RandomInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, HVResult &hvResult) {
    int n_ocs = geneticEvaluatorPtr->dp.oc_scores.size();
    hvResult.chromosome.resize(n_ocs);
    for(int i = 0;i<n_ocs;i++)
        hvResult.chromosome[i] = bernoulli_dist(rng);
    hvResult.cost = geneticEvaluatorPtr->evaluate_chromosome(hvResult.chromosome);
    hvResult.cost_history.emplace_back(hvResult.cost);
    hvResult.time=0;
}

