//
// Created by jens on 4/23/21.
//

#include "hypothesis_verification/hv_alg/bf.hpp"

chromosomeT BF::run() {
    if(geneticEvaluatorPtr!= nullptr) {
        best_chromosome.resize(geneticEvaluatorPtr->dp.ocs.size(),false);
        best_cost=std::numeric_limits<double>::max();

        BF_GEN bf_gen(geneticEvaluatorPtr->dp.ocs.size());
        while (bf_gen.next()){
            double cost = geneticEvaluatorPtr->evaluate_chromosome(bf_gen.chromosome);
            if(cost<best_cost){
                best_chromosome=bf_gen.chromosome;
                best_cost=cost;
            }
            std::cout<<bf_gen.n<<std::endl;
        }
        return best_chromosome;
    }
    else{
        std::cout<<"BF evaluator not initialized"<<std::endl;
        return best_chromosome;
    }
}
