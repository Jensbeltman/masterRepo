#ifndef MASTER_GA_FUNCTIONS_HPP
#define MASTER_GA_FUNCTIONS_HPP

#include <ga/typedefinitions.hpp>
#include <ga/ga.hpp>

chromosomeT crossover_uniform(GA *ga, int p1, int p2) {
    chromosomeT c;
    for (int i = 0; i < ga->n_genes; i++) {
        if (ga->bernoulli_dist(ga->rng)) {
            c.push_back(ga->last_population[p1][i]);
        } else {
            c.push_back(ga->last_population[p2][i]);
        }
    }
    return c;
}

void mutation_flip(GA *ga, chromosomeT &c) {
    for (int i = 0; i < ga->n_genes; i++) {
        if (ga->uniform_float_dist(ga->rng) < ga->mutation_rate) {
            c[i] = !c[i];
        }
    }
}


const auto crossover_default = crossover_uniform;
const auto mutation_default = mutation_flip;
#endif //MASTER_GA_FUNCTIONS_HPP
