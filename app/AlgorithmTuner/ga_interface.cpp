//
// Created by jens on 1/20/21.
//

#include "ga_interface.hpp"

GAInterface::GAInterface(): AlgorithmInterface() {
    name="GA";
    // Creating Variables
    variables_i.emplace_back(var_i{&ga.population_size,new QSpinBox,"population_size",std::to_string(ga.population_size)});
    variables_i.emplace_back(var_i{&ga.generation_max,new QSpinBox,"generation_max",std::to_string(ga.generation_max)});

    variables_d.emplace_back(var_d{&ga.mutation_rate,new QDoubleSpinBox,"mutation_rate",std::to_string(ga.mutation_rate)});
    variables_d.emplace_back(var_d{&ga.elite_pct,new QDoubleSpinBox,"elite_pct",std::to_string(ga.elite_pct)});
    variables_d.emplace_back(var_d{&ga.parent_pool_pct,new QDoubleSpinBox,"parent_pool_pct",std::to_string(ga.parent_pool_pct)});

    // Setting QT properties
    for(auto & var:variables_i)
        var.spinBox->setMaximum(10000);
}

void GAInterface::run(GeneticEvaluatorOCPtr &geneticEvaluatorOcPtr, std::vector<bool> &correct_ocs, std::vector<int> &tp,
                      std::vector<int> &fp, std::vector<int> &tn, std::vector<int> &fn) {
    GAResult gaResult;
    auto &dp = geneticEvaluatorOcPtr->dp;
    ga.n_genes = dp.ocs.size();
    ga.geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorOcPtr);
    gaResult = ga.solve();
    getFPTN(tp, fp, tn, fn, gaResult.chromosome, correct_ocs);
}
