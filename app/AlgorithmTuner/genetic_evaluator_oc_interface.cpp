//
// Created by jens on 1/28/21.
//

#include "genetic_evaluator_oc_interface.hpp"

GeneticEvaluatorInlierCollisionInterface::GeneticEvaluatorInlierCollisionInterface() {
    name="GEIC";
    geneticEvaluatorOcPtr = std::make_shared<GeneticEvaluatorInlierCollision>();
    geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorOcPtr);
    // Creating Variables
    parameters_d.emplace_back(param_d{&geneticEvaluatorOcPtr->nn_inlier_threshold, new QDoubleSpinBox, "NN inlier threshold", std::to_string(geneticEvaluatorOcPtr->nn_inlier_threshold)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorOcPtr->sigmoid_center, new QDoubleSpinBox, "Sigmoid center", std::to_string(geneticEvaluatorOcPtr->sigmoid_center)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorOcPtr->sigmoid_growth_rate, new QDoubleSpinBox, "Sigmoid growth rate", std::to_string(geneticEvaluatorOcPtr->sigmoid_growth_rate)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorOcPtr->oc_inlier_threshold, new QDoubleSpinBox, "Inlier threshold pct", std::to_string(geneticEvaluatorOcPtr->oc_inlier_threshold)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorOcPtr->vg_leaf_size, new QDoubleSpinBox, "VoxelGrid leaf size", std::to_string(geneticEvaluatorOcPtr->vg_leaf_size)});
}

GeneticEvaluatorInlierCollisionScaledInterface::GeneticEvaluatorInlierCollisionScaledInterface() {
    name="GEICS";
    geneticEvaluatorInlierCollisionScaledPtr = std::make_shared<GeneticEvaluatorInlierCollisionScaled>();
    geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorInlierCollisionScaledPtr);
    // Creating Variables
    parameters_d.emplace_back(param_d{&geneticEvaluatorInlierCollisionScaledPtr->nn_inlier_threshold, new QDoubleSpinBox, "NN inlier threshold", std::to_string(geneticEvaluatorInlierCollisionScaledPtr->nn_inlier_threshold)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorInlierCollisionScaledPtr->sigmoid_center, new QDoubleSpinBox, "Sigmoid center", std::to_string(geneticEvaluatorInlierCollisionScaledPtr->sigmoid_center)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorInlierCollisionScaledPtr->sigmoid_growth_rate, new QDoubleSpinBox, "Sigmoid growth rate", std::to_string(geneticEvaluatorInlierCollisionScaledPtr->sigmoid_growth_rate)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorInlierCollisionScaledPtr->oc_inlier_threshold, new QDoubleSpinBox, "Inlier threshold pct", std::to_string(geneticEvaluatorInlierCollisionScaledPtr->oc_inlier_threshold)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorInlierCollisionScaledPtr->vg_leaf_size, new QDoubleSpinBox, "VoxelGrid leaf size", std::to_string(geneticEvaluatorInlierCollisionScaledPtr->vg_leaf_size)});
}

GeneticEvaluatorScoreCollisionInterface::GeneticEvaluatorScoreCollisionInterface() {
    name="GESC";
    geneticEvaluatorScoreCollisionPtr = std::make_shared<GeneticEvaluatorScoreCollision>();
    geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorScoreCollisionPtr);
    // Creating Variables
    parameters_d.emplace_back(param_d{&geneticEvaluatorScoreCollisionPtr->nn_inlier_threshold, new QDoubleSpinBox, "NN inlier threshold", std::to_string(geneticEvaluatorScoreCollisionPtr->nn_inlier_threshold)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorScoreCollisionPtr->sigmoid_center, new QDoubleSpinBox, "Sigmoid center", std::to_string(geneticEvaluatorScoreCollisionPtr->sigmoid_center)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorScoreCollisionPtr->sigmoid_growth_rate, new QDoubleSpinBox, "Sigmoid growth rate", std::to_string(geneticEvaluatorScoreCollisionPtr->sigmoid_growth_rate)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorScoreCollisionPtr->oc_inlier_threshold, new QDoubleSpinBox, "Inlier threshold pct", std::to_string(geneticEvaluatorScoreCollisionPtr->oc_inlier_threshold)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorScoreCollisionPtr->vg_leaf_size, new QDoubleSpinBox, "VoxelGrid leaf size", std::to_string(geneticEvaluatorScoreCollisionPtr->vg_leaf_size)});
}

GeneticEvaluatorUniqueInlierCollisionScaledInterface::GeneticEvaluatorUniqueInlierCollisionScaledInterface() {
    geneticEvaluatorUniqueInlierCollisionScaledPtr = std::make_shared<GeneticEvaluatorUniqueInlierCollisionScaled>();
    name=geneticEvaluatorUniqueInlierCollisionScaledPtr->type;
    geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorUniqueInlierCollisionScaledPtr);
    // Creating Variables
    parameters_d.emplace_back(param_d{&geneticEvaluatorUniqueInlierCollisionScaledPtr->nn_inlier_threshold, new QDoubleSpinBox, "NN inlier threshold", std::to_string(geneticEvaluatorUniqueInlierCollisionScaledPtr->nn_inlier_threshold)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorUniqueInlierCollisionScaledPtr->sigmoid_center, new QDoubleSpinBox, "Sigmoid center", std::to_string(geneticEvaluatorUniqueInlierCollisionScaledPtr->sigmoid_center)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorUniqueInlierCollisionScaledPtr->sigmoid_growth_rate, new QDoubleSpinBox, "Sigmoid growth rate", std::to_string(geneticEvaluatorUniqueInlierCollisionScaledPtr->sigmoid_growth_rate)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorUniqueInlierCollisionScaledPtr->oc_inlier_threshold, new QDoubleSpinBox, "Inlier threshold pct", std::to_string(geneticEvaluatorUniqueInlierCollisionScaledPtr->oc_inlier_threshold)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorUniqueInlierCollisionScaledPtr->vg_leaf_size, new QDoubleSpinBox, "VoxelGrid leaf size", std::to_string(geneticEvaluatorUniqueInlierCollisionScaledPtr->vg_leaf_size)});
}

GeneticEvaluatorF1Interface::GeneticEvaluatorF1Interface() {
    geneticEvaluatorF1Ptr = std::make_shared<GeneticEvaluatorF1>();
    geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorF1Ptr);

    name=geneticEvaluatorF1Ptr->type;
    parameters_d.emplace_back(param_d{&geneticEvaluatorF1Ptr->t_thresh, new QDoubleSpinBox, "t_thresh", std::to_string(5)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorF1Ptr->r_thresh, new QDoubleSpinBox, "r_thresh", std::to_string(5)});
}

GeneticEvaluatorPrecisionInterface::GeneticEvaluatorPrecisionInterface() {
    geneticEvaluatorPrecisionPtr = std::make_shared<GeneticEvaluatorPrecision>();
    geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorPrecisionPtr);

    name=geneticEvaluatorPrecisionPtr->type;
    parameters_d.emplace_back(param_d{&geneticEvaluatorPrecisionPtr->t_thresh, new QDoubleSpinBox, "t_thresh", std::to_string(5)});
    parameters_d.emplace_back(param_d{&geneticEvaluatorPrecisionPtr->r_thresh, new QDoubleSpinBox, "r_thresh", std::to_string(5)});
}
