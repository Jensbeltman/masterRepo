#include "hv_interfaces.hpp"
#include "hypothesis_verification/hv_alg/ga_functions.hpp"
#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollisionVariants.hpp"
#include <cmath>

//BF
BFInterface::BFInterface() {
    name = "BF";
}

void BFInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, HVResult &hvResult) {
    chronometer.tic();
    bf.geneticEvaluatorPtr = geneticEvaluatorPtr;
    hvResult.chromosome = bf.run();
    hvResult.time = chronometer.toc();
    hvResult.cost = bf.best_cost;
}


//SP
SPInterface::SPInterface() : HVInterface() {
    name = "SP";
    sequentialPrior.score_threshold = 0;
}

void SPInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, HVResult &hvResult) {
    sequentialPrior.geneticEvaluatorPtr = geneticEvaluatorPtr;
    chronometer.tic();
    hvResult = sequentialPrior.solve();
    hvResult.time = chronometer.toc();
}

BLSPInterface::BLSPInterface() {
    name = "BLSP";
    parameters_d.emplace_back(param_d{&sequentialPrior.score_threshold, new QDoubleSpinBox, "Score Threshold",
                                      std::to_string(sequentialPrior.score_threshold)});
}

void BLSPInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, HVResult &hvResult) {
    SPInterface::run(geneticEvaluatorPtr, hvResult);
}

//SMC
SMCInterface::SMCInterface() {
    name = "SMC";
    sequentialMinCost.score_threshold = 0;
}

void SMCInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, HVResult &hvResult) {
    sequentialMinCost.geneticEvaluatorPtr = geneticEvaluatorPtr;
    chronometer.tic();
    hvResult = sequentialMinCost.solve();
    hvResult.time = chronometer.toc();
}

BLSMCInterface::BLSMCInterface() {
    name = "BLSP";
    parameters_d.emplace_back(param_d{&sequentialMinCost.score_threshold, new QDoubleSpinBox, "Score Threshold",
                                      std::to_string(sequentialMinCost.score_threshold)});
}

void BLSMCInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, HVResult &hvResult) {
    SMCInterface::run(geneticEvaluatorPtr, hvResult);
}


// GA
GAInterface::GAInterface() : HVInterface() {
    name = "GA";
    // Creating Variables
    parameters_i.emplace_back(param_i{&ga.population_size, getNewSpinBox(-10000, 10000), "population_size",
                                      std::to_string(ga.population_size)});
    parameters_i.emplace_back(param_i{&ga.generation_max, getNewSpinBox(-10000, 10000), "generation_max",
                                      std::to_string(ga.generation_max)});

    parameters_d.emplace_back(
            param_d{&ga.mutation_rate, new QDoubleSpinBox, "mutation_rate", std::to_string(ga.mutation_rate)});
    parameters_d.emplace_back(param_d{&ga.elite_pct, new QDoubleSpinBox, "elite_pct", std::to_string(ga.elite_pct)});
    parameters_d.emplace_back(
            param_d{&ga.parent_pool_pct, new QDoubleSpinBox, "parent_pool_pct", std::to_string(ga.parent_pool_pct)});

    parameters_b.emplace_back(param_b{&group_optim, new QCheckBox, "group_optim", std::to_string(group_optim)});
}

void GAInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, HVResult &hvResult) {

    GAResult gaResult;
    int n_ocs = geneticEvaluatorPtr->dp.ocs.size();
    ga.n_genes = n_ocs;
    ga.geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorPtr);
    int ga_population_size = ga.population_size;
    int ga_generation_max = ga.generation_max;

    if (group_optim && (geneticEvaluatorPtr->type == "GEUICS" || geneticEvaluatorPtr->type == "GELR")) {
        auto intersectingPointsPtr = std::dynamic_pointer_cast<IntersectingPoints>(geneticEvaluatorPtr);

        std::vector<int> group_idx(n_ocs, 0);

        int next_group_idx = 1;
        int pair_idx = 0;
        for (int i = 0; i < (n_ocs - 1); i++) {
            for (int j = i + 1; j < n_ocs; j++) {

                if (intersectingPointsPtr->collision_point_intersections[pair_idx] > 0) {
                    if (group_idx[i] == 0 && group_idx[j] == 0) {
                        group_idx[i] = next_group_idx;
                        group_idx[j] = next_group_idx;
                        next_group_idx++;
                    } else if (group_idx[i] != 0 && group_idx[j] == 0) {
                        group_idx[j] = group_idx[i];
                    } else if (group_idx[j] != 0 && group_idx[i] == 0) {
                        group_idx[i] = group_idx[j];
                    } else if (group_idx[i] != group_idx[j]) {
                        int il = std::min(group_idx[i], group_idx[j]);
                        int ih = std::max(group_idx[i], group_idx[j]);

                        for (auto &gi : group_idx) {
                            if (gi == ih)
                                gi = il;
                            if (gi > ih)
                                gi--;
                        }
                        next_group_idx--;
                    }
                }

                pair_idx++;
            }
        }

        std::vector<chromosomeT> masks(next_group_idx);
        for (auto &mask:masks)
            mask.resize(n_ocs, false);

        for (int i = 0; i < group_idx.size(); i++) {
            auto &gi = group_idx[i];
            masks[gi][i] = true;
        }

        hvResult.chromosome.resize(n_ocs, false);


        hvResult.time = 0;
        HVResult mask_result;


        for (int m = 0; m < masks.size(); m++) {
            auto &mask = masks[m];

            if (m == 0) {
                SequentialPrior sp;
                sp.geneticEvaluatorPtr = geneticEvaluatorPtr;
                geneticEvaluatorPtr->set_dp_mask(mask);
                chronometer.tic();
                mask_result = sp.solve();
                hvResult.time += chronometer.toc();
            } else {
                geneticEvaluatorPtr->set_dp_mask(mask);
                chronometer.tic();
                ga.population_size = ga_population_size;
                ga.generation_max = ga_generation_max;
                mask_result = ga.solve();
                hvResult.time += chronometer.toc();
            }

            for (int i = 0; i < n_ocs; i++) {
                if (mask[i]) {
                    hvResult.chromosome[i] = mask_result.chromosome[i];
                }
            }
        }
    } else {
        ga.population_size = ga_population_size;
        ga.generation_max = ga_generation_max;
        chronometer.tic();
        hvResult = ga.solve();
        hvResult.time = chronometer.toc();
    }

    chromosomeT full(n_ocs,true);
    ga.geneticEvaluatorPtr->set_dp_mask(full);
    hvResult.cost = ga.geneticEvaluatorPtr->evaluate_chromosome(hvResult.chromosome);


}

GAWInterface::GAWInterface() {
    name = "GAW";
    ga.generate_initial_population = initialize_population_score_sampling;
}


BaselineInterface::BaselineInterface() : HVInterface() {
    name = "BL";
    parameters_d.emplace_back(
            param_d{&score_threshold, new QDoubleSpinBox, "Score Threshold", std::to_string(score_threshold)});
}

void BaselineInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, HVResult &hvResult) {
    chronometer.tic();
    chromosomeT chromosome(geneticEvaluatorPtr->dp.ocs.size());
    for (int i = 0; i < chromosome.size(); i++)
        chromosome[i] = geneticEvaluatorPtr->dp.oc_scores[i] > score_threshold;

    hvResult.cost = geneticEvaluatorPtr->evaluate_chromosome(chromosome);
    hvResult.cost_history.emplace_back(hvResult.cost);
    hvResult.chromosome = chromosome;
    hvResult.time = chronometer.toc();
}

BLGAInterface::BLGAInterface() {
    name = "BLGA";
    parameters_d.emplace_back(
            param_d{&score_threshold, new QDoubleSpinBox, "Score Threshold", std::to_string(score_threshold)});
}

void BLGAInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, HVResult &hvResult) {
    chronometer.tic();
    chromosomeT mask(geneticEvaluatorPtr->n_ocs);
    for (int i = 0; i < mask.size(); i++)
        mask[i] = geneticEvaluatorPtr->dp.oc_scores[i] > score_threshold;

    geneticEvaluatorPtr->set_dp_mask(mask);


    GAInterface::run(geneticEvaluatorPtr, hvResult);
    for (int i = 0; i < mask.size(); i++) {
        if (!mask[i]) {
            if (hvResult.chromosome[i])
                std::cout << "Shitt" << std::endl;
            hvResult.chromosome[i] = false;
        }
    }

    std::fill(mask.begin(), mask.end(), true);
    geneticEvaluatorPtr->set_dp_mask(mask);

}


GASPInterface::GASPInterface() {
    name = "GASP";
    parameters_d.emplace_back(param_d{&sequentialPrior.score_threshold, new QDoubleSpinBox, "SP Score Threshold",
                                      std::to_string(sequentialPrior.score_threshold)});

}

void GASPInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, HVResult &hvResult) {
    chronometer.tic();
    // Get SP solution
    HVResult spResult;
    sequentialPrior.geneticEvaluatorPtr = geneticEvaluatorPtr;
    spResult = sequentialPrior.solve();

    GAResult gaResult;
    ga.n_genes = geneticEvaluatorPtr->dp.ocs.size();
    ga.geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorPtr);

    // Initialize GA based on SP
    ga.generate_initial_population = nullptr;
    ga.population.resize(ga.population_size);
    ga.population[0] = spResult.chromosome;
    for (int i = 1; i < ga.population_size; i++) {
        ga.mutation(&ga, spResult.chromosome);
        ga.population[i] = spResult.chromosome;
    }

    // solve with GA
    hvResult = ga.solve();
    hvResult.time = chronometer.toc();
}

RandomInterface::RandomInterface() {
    name = "RAND";

    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
    rng.seed(ss);
}

void RandomInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, HVResult &hvResult) {
    int n_ocs = geneticEvaluatorPtr->dp.oc_scores.size();
    hvResult.chromosome.resize(n_ocs);
    for (int i = 0; i < n_ocs; i++)
        hvResult.chromosome[i] = bernoulli_dist(rng);
    hvResult.cost = geneticEvaluatorPtr->evaluate_chromosome(hvResult.chromosome);
    hvResult.cost_history.emplace_back(hvResult.cost);
    hvResult.time = 0;
}

LogisticRegressionInterface::LogisticRegressionInterface() {
    name = "LR";
    parameters_d.emplace_back(
            param_d{&score_w, getNewDoubleSpinBox(-1000, 1000, 4), "score_w", std::to_string(score_w)});
    parameters_d.emplace_back(param_d{&visiblePointsFrac_w, getNewDoubleSpinBox(-1000, 1000, 4), "visiblePointsFrac_w",
                                      std::to_string(visiblePointsFrac_w)});
    parameters_d.emplace_back(param_d{&visibleInlierFrac_w, getNewDoubleSpinBox(-1000, 1000, 4), "visibleInlierFrac_w",
                                      std::to_string(visibleInlierFrac_w)});
    parameters_d.emplace_back(param_d{&penetration_w, getNewDoubleSpinBox(-1000, 1000, 4), "penetration_w",
                                      std::to_string(penetration_w)});
    parameters_d.emplace_back(
            param_d{&intersectingInliersFrac_w, getNewDoubleSpinBox(-1000, 1000, 4), "intersectingInliersFrac_w",
                    std::to_string(intersectingInliersFrac_w)});
    parameters_d.emplace_back(
            param_d{&intercept, getNewDoubleSpinBox(-1000, 1000, 4), "intercept", std::to_string(intercept)});
}

double LogisticRegressionInterface::sigmoid(double x) {
    return 1.0 / (1.0 + std::exp(-x));
}

void LogisticRegressionInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, HVResult &hvResult) {
    if (geneticEvaluatorPtr->type == "GEUICS" || geneticEvaluatorPtr->type == "GELR" ||
        geneticEvaluatorPtr->type == "GELRS") {
        auto gePtr = std::dynamic_pointer_cast<IntersectingPoints>(geneticEvaluatorPtr);
        auto &dp = gePtr->dp;
        hvResult.chromosome.resize(dp.ocs.size());
        std::fill(hvResult.chromosome.begin(), hvResult.chromosome.end(), true);
        std::vector<bool> inCollision;
        std::vector<double> penetration;
        std::vector<int> intersections;
        gePtr->get_max_collisions_in_chromosome(hvResult.chromosome, inCollision, penetration);
        gePtr->get_max_intersection_in_chromosome(hvResult.chromosome, intersections);

        auto &scores = dp.oc_scores;
        double model_points = gePtr->pcm->size();
        for (int i = 0; i < hvResult.chromosome.size(); i++) {
            double visiblePoints = gePtr->visible_oc_pcs[i]->size();
            double visibleInliers = gePtr->oc_visible_inlier_pt_idxs[i]->size();

            double x = scores[i] * score_w +
                       (visiblePoints / model_points) * visiblePointsFrac_w +
                       (visibleInliers / visiblePoints) * visibleInlierFrac_w +
                       penetration[i] * penetration_w +
                       (intersections[i] / visibleInliers) * intersectingInliersFrac_w
                       + intercept;
            x = sigmoid(x);
            hvResult.chromosome[i] = (x >= sigmoid_cutoff);
        }
    }
}

