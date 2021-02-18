//
// Created by jens on 1/20/21.
//

#include "algorithm_interface.hpp"
#include <QString>

// Algorithm
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


void AlgorithmInterface::add_variable_to_formlayout(QFormLayout *qFormLayout){
    for (auto &var:variables_b)
        qFormLayout->addRow(new QLabel(QString::fromStdString(var.name)), var.checkBox);
    for (auto &var:variables_i)
        qFormLayout->addRow(new QLabel(QString::fromStdString(var.name)), var.spinBox);
    for (auto &var:variables_d)
        qFormLayout->addRow(new QLabel(QString::fromStdString(var.name)), var.spinBox);
}

void AlgorithmInterface::add_variable_to_tabwidget(QTabWidget *qTabWidget){

        QWidget *widget = new QTabWidget();
        QFormLayout *qFormLayout = new QFormLayout();

        add_variable_to_formlayout(qFormLayout);

        widget->setLayout(qFormLayout);
        qTabWidget->addTab(widget, QString::fromStdString(name));

}
bool AlgorithmInterface::operator<(const AlgorithmInterface &rhs) const {
    return name < rhs.name;
}

bool AlgorithmInterface::operator==(const AlgorithmInterface &rhs) const {
    return name == rhs.name;
}

bool AlgorithmInterface::operator<(const std::string &rhs_string) const {
    return name < rhs_string;
}

bool AlgorithmInterface::operator==(const std::string &rhs_string) const {
    return name == rhs_string;
}

// HV
HVInterface::HVInterface() {
    variables_b.emplace_back(var_b{&enable,new QCheckBox,"enable","false"});
}

void HVInterface::getFPTN(std::vector<int> &tp, std::vector<int> &fp, std::vector<int> &tn, std::vector<int> &fn,
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

void HVInterface::run(GeneticEvaluatorPtr &geneticEvaluatorPtr, rawDataVecT &rawDataVec) {
    rawDataVec.emplace_back();
    run(geneticEvaluatorPtr,rawDataVec.back());
}

// Evaulator
