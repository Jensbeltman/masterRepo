#include "hypothesis_verification/hv_alg/ga.hpp"
#include "hypothesis_verification/hv_alg/ga_functions.hpp"
#include "hypothesis_verification/hv_alg/bf.hpp"
#include <iostream>
#include <ostream>


GA::GA(int n_genes, int population_size, int generation_max, double mutation_rate, double elite_pct,
       double parent_pool_pct) :
        n_genes(n_genes),
        population_size(population_size),
        generation_max(generation_max),
        mutation_rate(mutation_rate),
        elite_pct(elite_pct),
        parent_pool_pct(parent_pool_pct),
        generate_initial_population(population_initialization_default),
        crossover(crossover_default),
        mutation(mutation_default) {
}

void GA::initialize() {
    // Clear results
    result = GAResult();

    // Initialize mask if non exists
    if((geneticEvaluatorPtr->mask.size() != geneticEvaluatorPtr->dp.ocs.size()) || (geneticEvaluatorPtr->active_mask_size==0)) {
        chromosomeT full_mask(geneticEvaluatorPtr->dp.ocs.size(), true);
        geneticEvaluatorPtr->set_dp_mask(full_mask);
    }
    n_genes_in_mask = geneticEvaluatorPtr->active_mask_size;

    if (population_size<0){
        effective_population_size=abs(population_size)*n_genes_in_mask;
    }else{
        effective_population_size=population_size;
    }

    if(generation_max<0){
        effective_generation_max=static_cast<int>(std::abs(generation_max)*n_genes_in_mask);
    }else{
        effective_generation_max=generation_max;
    }


    // initialize the random number generator with time-dependent seed
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
    rng.seed(ss);

    // Set and validate parameters
    elite_cnt = std::max(1,static_cast<int>(elite_pct * effective_population_size));
    parent_pool_cnt = std::max(1,static_cast<int>(parent_pool_pct * effective_population_size));
    assert(effective_population_size || parent_pool_cnt);
    assert(elite_cnt <= effective_population_size);

    //Initialize vectors
    population_sorted_indices.resize(effective_population_size);
    std::iota(population_sorted_indices.begin(), population_sorted_indices.end(), 0);
    population_costs.resize(effective_population_size);
    std::fill(population_costs.begin(), population_costs.end(), std::numeric_limits<double>::infinity());

    uniform_dist_int_parent_pool = std::uniform_int_distribution<int>(0, parent_pool_cnt-1);
}

void GA::solve_init() {

    // If initial population generator specified generate population
    if (generate_initial_population != nullptr) {
        generate_initial_population(this);
    }
    // Prepare for solve loop
    calculate_population_cost(population_costs);
    last_population = population;
}

GAResult GA::solve() {
    initialize();
    if(! ((n_genes_in_mask<32) && (std::pow(2,n_genes_in_mask)<=(effective_generation_max*effective_population_size)))) { // 32 as this is the limit of the BF generator
        //std::cout<<"GA | pop size:"<<effective_population_size<<", gen max:"<<effective_generation_max<<", n_genes_in_mask:"<<n_genes_in_mask<<std::endl;

        solve_init();
        generation = 1;

        for (; generation < effective_generation_max; generation++) {
            population.clear();
            elite_transfer();
            crossover_and_mutation();
            calculate_population_cost(population_costs);
            last_population = population;
        }

        result.chromosome = population[population_sorted_indices[0]];


    }else{
        //std::cout<<"BF | pop size:"<<effective_population_size<<", gen max:"<<effective_generation_max<<", n_genes_in_mask:"<<n_genes_in_mask<<std::endl;
        BF bf;
        bf.geneticEvaluatorPtr = geneticEvaluatorPtr;
        result.chromosome = bf.run();
        result.cost = bf.best_cost;
    }
    return result;
}

void GA::elite_transfer() {
    for (int i = 0; i < elite_cnt; i++) {
        population.push_back(last_population[population_sorted_indices[i]]);
    }
}

int GA::select_parrent() {
    return population_sorted_indices[uniform_dist_int_parent_pool(rng)];
}

void GA::crossover_and_mutation() {
    for (int i = 0; i < effective_population_size - elite_cnt; i++) {
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

    result.cost_history.push_back(population_costs[population_sorted_indices[0]]);
    if (population_costs[population_sorted_indices[0]] < result.cost) {
        result.cost = population_costs[population_sorted_indices[0]];
        result.chromosome_index = population_sorted_indices[0];
    }
}


// Compare funcitons and ostream overloads
std::ostream &operator<<(std::ostream &os, const GAResult &result) {
    os << "Best chromosome " << result.chromosome << ", Cost " << result.cost;
    return os;
}

bool GA::cost_comp(int a, int b) {
    return population_costs[a] < population_costs[b];
}

