#ifndef MASTER_GENETICEVALUATOR_HPP
#define MASTER_GENETICEVALUATOR_HPP

#include "ga/utility/typedefinitions.hpp"

class GeneticEvaluator {
public:
    GeneticEvaluator();
    std::string type = "GeneticEvaluator";
    virtual double evaluate_chromosome(chromosomeT &chromosome) = 0;
};


#endif //MASTER_GENETICEVALUATOR_HPP
