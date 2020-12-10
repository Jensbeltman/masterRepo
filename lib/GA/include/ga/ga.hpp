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
    chromosomeT chromosome;
    int chromosome_index = 0;
    double cost = std::numeric_limits<double>::max();
    std::vector<double> cost_history;
};

std::ostream &operator<<(std::ostream &os, const GAResult &result);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(GAResult, chromosome, chromosome_index, cost,
                                   cost_history)


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
    int n_genes;
    int population_size;
    int generation_max;
    float mutation_rate;
    double elite_pct;
    double parent_pool_pct;


    int generation;

    //functions
    GA(int n_genes = 100, int population_size = 100, int generation_max = 100, double mutation_rate = 0.05,
       double elite_pct = 0.1, double parent_pool_pct = 0.3);

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

    int elite_cnt;
    int parent_pool_cnt;

    bool cost_comp(int a, int b);

};


#endif //MASTER_GA_HPP
