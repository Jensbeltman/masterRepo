//
// Created by jens on 5/14/21.
//

#include "hypothesis_verification/hv_alg/sequential_min_cost.hpp"

SequentialMinCost::SequentialMinCost(GeneticEvaluatorPtr geneticEvaluatorPtr):geneticEvaluatorPtr(geneticEvaluatorPtr) {

}

HVResult SequentialMinCost::solve() {
    // BASELINE
    HVResult result;
    std::vector<T4> &object_candidates = geneticEvaluatorPtr->dp.ocs;
    std::vector<T4> &gts = geneticEvaluatorPtr->dp.gts;
    std::vector<double> &object_candidates_scores = geneticEvaluatorPtr->dp.oc_scores;

    result.chromosome = chromosomeT(object_candidates.size(), false);
    auto & chromosome =  result.chromosome;

    double start_cost = geneticEvaluatorPtr->evaluate_chromosome(result.chromosome);
    double& cost = result.cost;
    cost = start_cost;
    for (int i = 0;i<object_candidates.size();i++) {
        int best_i_to_add = -1;
        double best_cost_to_add = cost;
        if(object_candidates_scores[i]>score_threshold) {
            {
                for (int j = 0;j<object_candidates.size();j++) {
                    if(!chromosome[j]) {
                        chromosome[j] = true;
                        double cost_by_adding_j = geneticEvaluatorPtr->evaluate_chromosome(chromosome);
                        chromosome[j] = false;
                        if (cost_by_adding_j < best_cost_to_add) {
                            best_i_to_add = j;
                            best_cost_to_add = cost_by_adding_j;
                        }
                    }
                }
                if(best_i_to_add != -1) {
                    chromosome[best_i_to_add] = true;
                    result.cost_history.emplace_back(best_cost_to_add);
                    cost=best_cost_to_add;
                }
            }
        }
    }
    return result;
}
