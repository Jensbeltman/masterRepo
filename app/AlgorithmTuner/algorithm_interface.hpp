//
// Created by jens on 1/20/21.
//

#ifndef MASTER_ALGORITHM_INTERFACE_HPP
#define MASTER_ALGORITHM_INTERFACE_HPP
#include "tuple"
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QSettings>
#include "ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp"

struct var_b{
   bool* val;
   QCheckBox* checkBox;
   std::string name;
   std::string default_val_string = "false";
};
struct var_i{
   int* val;
   QSpinBox* spinBox;
   std::string name;
   std::string default_val_string = "0";
};
struct var_d{
   double* val;
   QDoubleSpinBox* spinBox;
   std::string name;
   std::string default_val_string = "0.0";
};


class AlgorithmInterface {
public:
    AlgorithmInterface();

    std::string name;
    bool enable=false;
    std::vector<var_b> variables_b;
    std::vector<var_i> variables_i;
    std::vector<var_d> variables_d;

    void update_variables();
    void load_settings(QSettings& qsettings);
    void save_settings(QSettings& qsettings);

    bool enabled();

    virtual void run(GeneticEvaluatorOCPtr &geneticEvaluatorOcPtr, std::vector<bool> &correct_ocs, std::vector<int> &tp, std::vector<int> &fp, std::vector<int> &tn, std::vector<int> &fn);

protected:
    QVariant get_setting_variant(QSettings &qsettings, std::string key, std::string default_val);

    void getFPTN(std::vector<int> &tp, std::vector<int> &fp, std::vector<int> &tn, std::vector<int> &fn, chromosomeT chromosome,
            chromosomeT correct_ocs);

/*
    template<typename dT, typename vT>
    dT* get_variable(std::string variable_name,std::vector<vT> variables){
        for(auto& var:variables) {
            auto found_variable = std::find_if(variables.begin(),variables.end(),[&variable_name](vT &vt){return vt.name == variable_name;});
            if(found_variable!=variables.end())
                return ((*found_variable).val);
        }
        return nullptr
    }
*/


};

typedef std::shared_ptr<AlgorithmInterface> AlgorithmInterfacePtr;

#endif //MASTER_ALGORITHM_INTERFACE_HPP
