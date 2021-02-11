#ifndef MASTER_GA_FUNCTIONS_HPP
#define MASTER_GA_FUNCTIONS_HPP

#include "../typedefinitions.hpp"
#include "ga.hpp"

chromosomeT crossover_uniform(GA *ga, int p1, int p2);
void mutation_flip(GA *ga, chromosomeT &c);
void initialize_population_bernoulli(GA* ga);
void initialize_population_score_sampling(GA* ga);

const auto crossover_default = crossover_uniform;
const auto mutation_default = mutation_flip;
const auto population_initialization_default = initialize_population_bernoulli;
#endif //MASTER_GA_FUNCTIONS_HPP
