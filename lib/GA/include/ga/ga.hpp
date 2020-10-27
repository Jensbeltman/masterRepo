#ifndef MASTER_GA_HPP
#define MASTER_GA_HPP

#include <random>
#include <chrono>
#include <thread>
#include <ctime>
#include <string>
#include <iostream>
#include <algorithm>
#include <functional>
#include <mutex>
#include <atomic>
#include <vector>
#include "ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp"
#include <nlohmann/json.hpp>
#include "typedefinitions.hpp"


struct GAResult {
    chromosomeT best_chromosome;
    int best_chromosome_index = 0;
    double best_chromosome_cost = std::numeric_limits<double>::max();
    std::vector<double> best_chromosome_cost_history;
};
std::ostream &operator<<(std::ostream &os, const GAResult& result);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(GAResult, best_chromosome, best_chromosome_index, best_chromosome_cost,
                                   best_chromosome_cost_history)


class GA {
public:
    // random generators and distributions
    std::mt19937_64 rng; // random generator
    std::uniform_real_distribution<double> uniform_float_dist;
    std::uniform_int_distribution<int> uniform_dist_int_parent_pool;
    std::bernoulli_distribution bernoulli_dist;

    populationT population;
    populationT last_population;
    std::vector<int> population_sorted_indices; // sorted based on cost
    std::vector<double> population_costs; // cost of population chromosomes

    GAResult result;


    // Params
    int N_genes;
    int N_chromosomes;
    int generation_max;
    float mutation_rate;
    int elite_count;
    int parent_pool_count;

    int N_threads = 0;
    int generation;

    //functions
    GA(int N_genes);

    void calculate_population_cost(std::vector<double> &costs);

    void elite_transfer();

    void crossover_and_mutation();

    int select_parrent();

    std::function<void(GA *)> generate_initial_population;
    std::function<chromosomeT(GA *, int p1, int p2)> crossover;
    std::function<void(GA *, chromosomeT &c)> mutation;

    void solve_init();

    GAResult solve();

    std::shared_ptr<GeneticEvaluator> geneticEvaluatorPtr;

    //Private functions
private:
    void initialize();

    bool cost_comp(int a, int b);

};


#endif //MASTER_GA_HPP
