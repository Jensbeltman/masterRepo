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
#include <QFormLayout>
#include <QLabel>
#include <QWidget>
#include <QTabWidget>
#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollision.hpp"
#include "hypothesis_verification/hv_alg/hv_result.hpp"
#include "algorithm_interface.hpp"
#include "chronometer.h"

struct param_b{
   bool* val;
   QCheckBox* checkBox;
   std::string name;
   std::string default_val_string = "false";
};
struct param_i{
   int* val;
   QSpinBox* spinBox;
   std::string name;
   std::string default_val_string = "0";
};
struct param_d{
   double* val;
   QDoubleSpinBox* spinBox;
   std::string name;
   std::string default_val_string = "0.0";
};

struct rawDataT{
    std::string alg_name;
    DatasetObjectPtr objPtr;
    int dpI;
    HVResult hvResult;
    double time;
};
typedef std::vector<rawDataT> rawDataVecT;
typedef std::map<DatasetObjectPtr, rawDataVecT> rawDataMapObjVecT;
typedef std::map<std::string,rawDataMapObjVecT> rawDataMapAlgObjVecT;

class AlgorithmInterface{
public:
    AlgorithmInterface()=default;
    std::string name;

    std::vector<param_b> parameters_b;
    std::vector<param_i> parameters_i;
    std::vector<param_d> parameters_d;

    Chronometer chronometer;
    void update_parameters();
    void load_settings(QSettings& qsettings);
    void save_settings(QSettings& qsettings);

    void add_parameters_to_formlayout(QFormLayout *qFormLayout);
    void add_parameters_to_tabwidget(QTabWidget *qTabWidget);

    bool operator <(const AlgorithmInterface& rhs) const;
    bool operator ==(const AlgorithmInterface& rhs) const;
    bool operator <(const std::string& rhs_string) const;
    bool operator ==(const std::string& rhs_string) const;


protected:
    QVariant get_setting_variant(QSettings &qsettings, std::string key, std::string default_val);
};

class HVInterface: public AlgorithmInterface {
public:
    HVInterface();

    bool enabled = false;

    virtual void run(GeneticEvaluatorPtr &geneticEvaluatorPtr,HVResult &hvResult) = 0;


protected:
    void getFPTN(std::vector<int> &tp, std::vector<int> &fp, std::vector<int> &tn, std::vector<int> &fn, chromosomeT chromosome,
            chromosomeT correct_ocs);

};


class EvaluatorInterface: public AlgorithmInterface {
public:
    EvaluatorInterface()=default;
    GeneticEvaluatorPtr geneticEvaluatorPtr;
};


typedef std::shared_ptr<AlgorithmInterface> AlgorithmInterfacePtr;
typedef std::shared_ptr<HVInterface> HVInterfacePtr;
typedef std::shared_ptr<EvaluatorInterface> EvaluatorInterfacePtr;

#endif //MASTER_ALGORITHM_INTERFACE_HPP
