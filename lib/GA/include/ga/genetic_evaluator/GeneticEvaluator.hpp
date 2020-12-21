#ifndef MASTER_GENETICEVALUATOR_HPP
#define MASTER_GENETICEVALUATOR_HPP

#include "ga/typedefinitions.hpp"

class GeneticEvaluator {
public:
    GeneticEvaluator();

    std::string type = "GeneticEvaluator";

    virtual double evaluate_chromosome(chromosomeT &chromosome) = 0;
    virtual double evaluate_chromosome(chromosomeT &chromosome,std::vector<double> coefficients) = 0;
    virtual void getHyperParameters_d(std::vector<std::string> names, std::vector<double*> params);
    virtual void setHyperParameters_d(std::vector<double> &params);
    virtual void getHyperParameters_i(std::vector<std::string> names, std::vector<int*> params);
    virtual void setHyperParameters_i(std::vector<int> &params);
};


#endif //MASTER_GENETICEVALUATOR_HPP
