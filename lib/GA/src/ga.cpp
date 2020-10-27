#include "ga/ga.hpp"
#include <iostream>
#include <ostream>


GA::GA(int N_genes) :
        N_genes(N_genes),
        N_chromosomes(100),
        generation_max(100),
        mutation_rate(0.005),
        elite_count((int) (0.1 * N_genes)),
        parent_pool_count((int) (0.3 * N_genes)),
        generate_initial_population(nullptr),
        crossover(nullptr),
        mutation(nullptr) {
    // initialize the random number generator with time-dependent seed
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
    rng.seed(ss);

    N_threads = std::thread::hardware_concurrency();// read number of threads

    if (N_threads == 0) // number of CPU cores not detected.
        N_threads = 8;
}

void GA::initialize() {
    // Clear results
    result = GAResult();

    // Validate parameters
    assert(N_chromosomes || parent_pool_count);
    assert(elite_count <= N_chromosomes);

    //Initialize vectors
    population_sorted_indices.resize(N_chromosomes);
    std::iota(population_sorted_indices.begin(), population_sorted_indices.end(), 0);
    population_costs.resize(N_chromosomes);
    std::fill(population_costs.begin(), population_costs.end(), std::numeric_limits<double>::infinity());

    uniform_dist_int_parent_pool = std::uniform_int_distribution<int>(0, parent_pool_count);


}

void GA::solve_init() {
    // If initial population generator specified generate population
    if (generate_initial_population != nullptr) {
        generate_initial_population(this);
    }

    // Generate random initial population
    population.resize(N_chromosomes);
    for (chromosomeT &c: population) {
        c.clear();
        for (int i = 0; i < N_genes; i++) {
            bool gv = bernoulli_dist(rng);
            c.push_back(gv);
        }
    }

    // Prepare for solve loop
    calculate_population_cost(population_costs);
    last_population = population;
}

GAResult GA::solve() {
    initialize();
    solve_init();
    generation = 1;

    for (; generation < generation_max; generation++) {
        population.clear();
        elite_transfer();
        crossover_and_mutation();
        calculate_population_cost(population_costs);
        last_population = population;
    }

    result.best_chromosome = population[population_sorted_indices[0]];

    return result;
}

void GA::elite_transfer() {
    for (int i = 0; i < elite_count; i++) {
        population.push_back(last_population[population_sorted_indices[i]]);
    }
}

int GA::select_parrent() {
    return population_sorted_indices[uniform_dist_int_parent_pool(rng)];
}

void GA::crossover_and_mutation() {
    for (int i = 0; i < N_chromosomes - elite_count; i++) {
        int p1i = select_parrent();
        int p2i = select_parrent();
        chromosomeT c = crossover(this, p1i, p2i);
        mutation(this, c);
        population.push_back(c);
    }
}
//Todo consider if creating two children(one inverse of the other) might be more efficient since the random generation then will be used less
// Ostream Overloads


void GA::calculate_population_cost(std::vector<double> &costs) {
    for (int i = 0; i < population.size(); i++) {
        costs[i] = geneticEvaluatorPtr->evaluate_chromosome(population[i]);
    }

    sort(population_sorted_indices.begin(), population_sorted_indices.end(),
         [&costs](int a, int b) -> bool {
             return costs[a] < costs[b];
         });

    result.best_chromosome_cost_history.push_back(population_costs[population_sorted_indices[0]]);
    if (population_costs[population_sorted_indices[0]] < result.best_chromosome_cost) {
        result.best_chromosome_cost = population_costs[population_sorted_indices[0]];
        result.best_chromosome_index = population_sorted_indices[0];
    }
}


// Compare funcitons and ostream overloads
std::ostream &operator<<(std::ostream &os, const GAResult& result){
    os<<"Best chromosome "<<result.best_chromosome<<", Cost "<<result.best_chromosome_cost;
    return os;
}

bool GA::cost_comp(int a, int b) {
    return population_costs[a] < population_costs[b];
}

