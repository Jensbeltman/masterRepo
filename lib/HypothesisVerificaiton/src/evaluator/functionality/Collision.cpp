#include "hypothesis_verification/evaluator/functionality/Collision.hpp"
#include "hypothesis_verification/evaluator/collision_checking.hpp"

void Collision::init_collisions() {
    // Detect collisions
    chronometer.tic();
    collisions.pairs.clear();
    collisions = get_collisions(dp.ocs, meshPtr);
    std::cout << "Collision init elapsed time: " << chronometer.toc() << "s\n";

}

void Collision::get_max_collisions_in_chromosome(chromosomeT &chromosome, std::vector<bool> &in_collision, std::vector<double> &penetration) {
    // This version calculated the maximum collision depth for a given oc
    in_collision.resize(chromosome.size(),false);
    penetration.resize(chromosome.size(),0);
    for (int i = 0; i < collisions.pairs.size(); i++){
        auto &cp = collisions.pairs[i];
        double &pen_dist = collisions.distances[i];
        if (chromosome[cp.first] && chromosome[cp.second]) {
            in_collision[cp.first] = true;
            in_collision[cp.second] = true;

            penetration[cp.first] = std::max(penetration[cp.first], pen_dist);
            penetration[cp.second] = std::max(penetration[cp.second], pen_dist);
        }
    }
}
