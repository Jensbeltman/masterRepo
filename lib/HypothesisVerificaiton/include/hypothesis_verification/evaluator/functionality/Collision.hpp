#ifndef MASTER_COLLISION_HPP
#define MASTER_COLLISION_HPP
#include "hypothesis_verification/evaluator/GeneticEvaluator.hpp"
#include "hypothesis_verification/evaluator/collision_checking.hpp"

class Collision: virtual public GeneticEvaluator {
public:
    void init_collisions();
    void get_max_collisions_in_chromosome(chromosomeT &chromosome, std::vector<bool> &in_collision, std::vector<double> &penetration);
    Collisions collisions;
    std::vector<std::pair<int, int>> oc_collision_pairs;
    std::vector<double> oc_collision_pair_penetration;
};


#endif //MASTER_COLLISION_HPP
