#include "hypothesis_verification/hv_alg/ga_functions.hpp"
chromosomeT crossover_uniform(GA *ga, int p1, int p2) {
    chromosomeT c;
    for (int i = 0; i < ga->n_genes; i++) {
        if(ga->geneticEvaluatorPtr->mask[i]) {
            if (ga->bernoulli_dist(ga->rng)) {
                c.push_back(ga->last_population[p1][i]);
            } else {
                c.push_back(ga->last_population[p2][i]);
            }
        }
        else{
            c.push_back(false);
        }
    }
    return c;
}

void mutation_flip(GA *ga, chromosomeT &c) {
    for (int i = 0; i < ga->n_genes; i++) {
        if(ga->geneticEvaluatorPtr->mask[i]) {
            if (ga->uniform_float_dist(ga->rng) < ga->mutation_rate) {
                c[i] = !c[i];
            }
        }
    }
}

void initialize_population_bernoulli(GA* ga){
    ga->population.resize(ga->population_size);
    for (chromosomeT &c: ga->population) {
        c.clear();
        for (int i = 0; i < ga->n_genes; i++) {
            if(ga->geneticEvaluatorPtr->mask[i]) {
                bool gv = ga->bernoulli_dist(ga->rng);
                c.push_back(gv);
            }else{
                c.push_back(false);
            }
        }
    }
    std::fill(ga->population.front().begin(),ga->population.front().end(),true);
    std::fill(ga->population.back().begin(),ga->population.back().end(),false);
}

void initialize_population_score_sampling(GA* ga){
    auto oc_scores = ga->geneticEvaluatorPtr->dp.oc_scores;
    double min = *std::min_element(oc_scores.begin(),oc_scores.end());
    std::for_each(oc_scores.begin(),oc_scores.end(),[min](double &val){val-=min;});
    double score_sum = std::accumulate(oc_scores.begin(),oc_scores.end(),0.0);


    double cummulative_sum;
    for(auto&val:oc_scores){
        cummulative_sum+=val;
        val = cummulative_sum/score_sum;
    }

    ga->population.resize(ga->population_size);
    for (chromosomeT &c: ga->population) {
        c.clear();
        for (int i = 0; i < ga->n_genes; i++) {
            c.push_back(oc_scores[i]<ga->uniform_float_dist(ga->rng));
        }
    }
}