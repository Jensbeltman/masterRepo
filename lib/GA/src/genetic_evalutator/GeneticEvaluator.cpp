//
// Created by jens on 10/6/20.
//

#include "ga/genetic_evaluator/GeneticEvaluator.hpp"

GeneticEvaluator::GeneticEvaluator() {

}

void GeneticEvaluator::getHyperParameters_d(std::vector <std::string> names, std::vector<double *> params) {
    std::cout<<"GeneticEvaluator::getHyperParameters_d was called. Is a baseclass with no functionality"<<std::endl;
    return;
}

void GeneticEvaluator::setHyperParameters_d(std::vector<double> &params) {
    std::cout<<"GeneticEvaluator::setHyperParameters_d was called. Is a baseclass with no functionality"<<std::endl;
    return;
}

void GeneticEvaluator::getHyperParameters_i(std::vector <std::string> names, std::vector<int *> params) {
    std::cout<<"GeneticEvaluator::getHyperParameters_i was called. Is a baseclass with no functionality"<<std::endl;
    return;
}

void GeneticEvaluator::setHyperParameters_i(std::vector<int> &params) {
    std::cout<<"GeneticEvaluator::setHyperParameters_i was called. Is a baseclass with no functionality"<<std::endl;
    return;
}
