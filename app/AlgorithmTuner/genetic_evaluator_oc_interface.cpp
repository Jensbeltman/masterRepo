//
// Created by jens on 1/28/21.
//

#include "genetic_evaluator_oc_interface.hpp"

GeneticEvaluatorOCInterface::GeneticEvaluatorOCInterface() {
    name="GeneticEvaluatorOC";
    geneticEvaluatorOcPtr = std::make_shared<GeneticEvaluatorOC>();
    geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorOcPtr);
    // Creating Variables
    variables_d.emplace_back(var_d{&geneticEvaluatorOcPtr->nn_inlier_threshold,new QDoubleSpinBox,"NN inlier threshold",std::to_string(geneticEvaluatorOcPtr->nn_inlier_threshold)}); ;
    variables_d.emplace_back(var_d{&geneticEvaluatorOcPtr->sigmoid_falloff_center,new QDoubleSpinBox,"Sigmoid center",std::to_string(geneticEvaluatorOcPtr->sigmoid_falloff_center)}); ;
    variables_d.emplace_back(var_d{&geneticEvaluatorOcPtr->sigmoid_falloff_scale,new QDoubleSpinBox,"Sigmoid scale",std::to_string(geneticEvaluatorOcPtr->sigmoid_falloff_scale)}); ;
    variables_d.emplace_back(var_d{&geneticEvaluatorOcPtr->oc_inlier_threshold,new QDoubleSpinBox,"Inlier threshold pct",std::to_string(geneticEvaluatorOcPtr->oc_inlier_threshold)}); ;
    variables_d.emplace_back(var_d{&geneticEvaluatorOcPtr->vg_leaf_size, new QDoubleSpinBox, "VoxelGrid leaf size", std::to_string(geneticEvaluatorOcPtr->vg_leaf_size)}); ;
}
