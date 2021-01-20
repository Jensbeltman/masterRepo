//
// Created by jens on 1/20/21.
//

#include "algorithm_interface.hpp"
#include <QString>

AlgorithmInterface::AlgorithmInterface() {
    variables_b.emplace_back(var_b{&enable,new QCheckBox,"enable","false"});
}

void AlgorithmInterface::update_variables() {
    for(auto& var:variables_b){
        *var.val = var.checkBox->isChecked();
    }

    for(auto& var:variables_i){
        *var.val = var.spinBox->value();
    }

    for(auto& var:variables_d){
        *var.val = var.spinBox->value();
    }
}

void AlgorithmInterface::load_settings(QSettings& qsettings) {
    for(auto& var:variables_b){
        *var.val = get_setting_variant(qsettings,name + "/" + var.name, var.default_val_string).toBool();
        var.checkBox->setChecked(*var.val);
    }

    for(auto& var:variables_i){
        *var.val = get_setting_variant(qsettings,name + "/" + var.name, var.default_val_string).toInt();
        var.spinBox->setValue(*var.val);
    }

    for(auto& var:variables_d){
        *var.val = get_setting_variant(qsettings,name + "/" + var.name, var.default_val_string).toDouble();
        var.spinBox->setValue(*var.val);
    }
}

void AlgorithmInterface::save_settings(QSettings& qsettings) {
    for(auto& var:variables_b){
      qsettings.setValue(QString::fromStdString(name + "/" + var.name), var.checkBox->isChecked());
    }
    for(auto& var:variables_d){
        qsettings.setValue(QString::fromStdString(name + "/" + var.name), var.spinBox->value());
    }
    for(auto& var:variables_i){
        qsettings.setValue(QString::fromStdString(name + "/" + var.name), var.spinBox->value());
    }
}

QVariant AlgorithmInterface::get_setting_variant(QSettings &qsettings, std::string key, std::string default_val) {
    return qsettings.value(QString::fromStdString(key),QString::fromStdString(default_val));
}

void AlgorithmInterface::run(GeneticEvaluatorOCPtr &geneticEvaluatorOcPtr, std::vector<bool> &correct_ocs,
                             std::vector<int> &tp, std::vector<int> &fp, std::vector<int> &tn, std::vector<int> &fn) {

}

void AlgorithmInterface::getFPTN(std::vector<int> &tp, std::vector<int> &fp, std::vector<int> &tn, std::vector<int> &fn,
                                 chromosomeT chromosome, chromosomeT correct_ocs) {
    for (int i = 0; i < correct_ocs.size(); i++) {
        if (correct_ocs[i]) {
            if (chromosome[i]) {
                tp.emplace_back(i);
            } else {
                fn.emplace_back(i);
            }
        } else {
            if (chromosome[i]) {
                fp.emplace_back(i);
            } else {
                tn.emplace_back(i);
            }
        }
    }
}

bool AlgorithmInterface::enabled() {
    auto enabled_var_it = std::find_if(variables_b.begin(),variables_b.end(),[](var_b vb){return vb.name=="enable";});
    if( enabled_var_it != variables_b.end())
        return (*enabled_var_it).val;
    else
        return false;
}

