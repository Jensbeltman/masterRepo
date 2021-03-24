//
// Created by jens on 1/20/21.
//

#include "algorithm_interface.hpp"
#include <QString>

// Algorithm
void AlgorithmInterface::update_parameters() {
    for(auto& var:parameters_b){
        *var.val = var.checkBox->isChecked();
    }

    for(auto& var:parameters_i){
        *var.val = var.spinBox->value();
    }

    for(auto& var:parameters_d){
        *var.val = var.spinBox->value();
    }
}

void AlgorithmInterface::load_settings(QSettings& qsettings) {
    for(auto& var:parameters_b){
        *var.val = get_setting_variant(qsettings,name + "/" + var.name, var.default_val_string).toBool();
        var.checkBox->setChecked(*var.val);
    }

    for(auto& var:parameters_i){
        *var.val = get_setting_variant(qsettings,name + "/" + var.name, var.default_val_string).toInt();
        var.spinBox->setValue(*var.val);
    }

    for(auto& var:parameters_d){
        *var.val = get_setting_variant(qsettings,name + "/" + var.name, var.default_val_string).toDouble();
        var.spinBox->setValue(*var.val);
    }
}

void AlgorithmInterface::save_settings(QSettings& qsettings) {
    for(auto& var:parameters_b){
      qsettings.setValue(QString::fromStdString(name + "/" + var.name), var.checkBox->isChecked());
    }
    for(auto& var:parameters_d){
        qsettings.setValue(QString::fromStdString(name + "/" + var.name), var.spinBox->value());
    }
    for(auto& var:parameters_i){
        qsettings.setValue(QString::fromStdString(name + "/" + var.name), var.spinBox->value());
    }
}

QVariant AlgorithmInterface::get_setting_variant(QSettings &qsettings, std::string key, std::string default_val) {
    return qsettings.value(QString::fromStdString(key),QString::fromStdString(default_val));
}


void AlgorithmInterface::add_parameters_to_formlayout(QFormLayout *qFormLayout){
    for (auto &var:parameters_b)
        qFormLayout->addRow(new QLabel(QString::fromStdString(var.name)), var.checkBox);
    for (auto &var:parameters_i)
        qFormLayout->addRow(new QLabel(QString::fromStdString(var.name)), var.spinBox);
    for (auto &var:parameters_d) {
        if( var.spinBox== nullptr)
            std::cout<<"shit"<<std::endl;
        qFormLayout->addRow(new QLabel(QString::fromStdString(var.name)), var.spinBox);
    }
}

void AlgorithmInterface::add_parameters_to_tabwidget(QTabWidget *qTabWidget){

        QWidget *widget = new QTabWidget();
        QFormLayout *qFormLayout = new QFormLayout();

    add_parameters_to_formlayout(qFormLayout);

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
    parameters_b.emplace_back(param_b{&enabled, new QCheckBox, "enable", "false"});
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

QSpinBox* getNewSpinBox(int min, int max) {
    auto spinBox = new QSpinBox;
    spinBox->setMinimum(min);
    spinBox->setMaximum(max);
    return spinBox;
}

QDoubleSpinBox* getNewDoubleSpinBox(double min, double max, int decimals) {
    auto spinBox = new QDoubleSpinBox;
    spinBox->setMinimum(min);
    spinBox->setMaximum(max);
    spinBox->setDecimals(decimals);
    return spinBox;
}
