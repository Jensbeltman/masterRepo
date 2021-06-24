//
// Created by jens on 4/23/21.
//

#include "hypothesis_verification/hv_alg/bf.hpp"

chromosomeT BF::run() {
    if(geneticEvaluatorPtr!= nullptr) {
        chromosomeT chromosome(geneticEvaluatorPtr->dp.ocs.size(),false);
        best_chromosome.resize(geneticEvaluatorPtr->dp.ocs.size(),false);
        best_cost=std::numeric_limits<double>::max();
        std::vector<int> mask_idx;
        for(int i = 0;i<geneticEvaluatorPtr->dp.ocs.size();i++){
            if(geneticEvaluatorPtr->mask[i]){
                mask_idx.emplace_back(i);
            }
        }

        BF_GEN bf_gen(geneticEvaluatorPtr->active_mask_size);
        while (bf_gen.next()){

            for(int i =0;i<mask_idx.size();i++)
                chromosome[mask_idx[i]]=bf_gen.chromosome[i];

            double cost = geneticEvaluatorPtr->evaluate_chromosome(chromosome);
            if(cost<best_cost){
                best_chromosome=chromosome;
                best_cost=cost;
            }
        }
        return best_chromosome;
    }
    else{
        std::cout<<"BF evaluator not initialized"<<std::endl;
        return best_chromosome;
    }
}
