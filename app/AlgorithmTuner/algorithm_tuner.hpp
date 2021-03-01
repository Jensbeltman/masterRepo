#ifndef MASTER_ALGORITHM_TUNER_HPP
#define MASTER_ALGORITHM_TUNER_HPP

#include <ui_algorithm_tuner.h>

#include <QMainWindow>
#include <QMutex>
#include <QTimer>
#include <QSettings>
#include <QFileSystemModel>
#include <QDockWidget>
#include <QCheckBox>
#include <QCoreApplication>
#include <QDebug>
#include <QMetaType>
#include <QSettings>
#include <QVariant>
#include "algorithm_interface.hpp"
#include "hv_interfaces.hpp"
#include "genetic_evaluator_oc_interface.hpp"
#include "dataset/DatasetObject.hpp"
#include <dataset/scape/ScapeDataset.hpp>
#include "hypothesis_verification/visualization/point_cloud_group_visualizer.hpp"
#include "hypothesis_verification/evaluator/GeneticEvaluatorInlierCollision.hpp"
#include "vtkSmartPointer.h"
#include "algorithm_data_proc.hpp"



struct AlgorithmTunerSettings {
    QString data_folder;
    QString recognition_folder;
    QString data_save_file;
    QString tuned_params_file;
};

struct GeneralSettings {
    QDoubleSpinBox *ground_truth_t_thresh = new QDoubleSpinBox();
    QDoubleSpinBox *ground_truth_r_thresh = new QDoubleSpinBox();
};

struct ObjectTunedVariables{
    std::string alg_name;
    std::string obj_name;
    std::string var_name;
    double value;
};

namespace Ui {
    class AlgorithmTuner;
}

class AlgorithmTuner : public QMainWindow, private Ui::AlgorithmTuner {
Q_OBJECT
public:
    explicit AlgorithmTuner(QMainWindow *parent = nullptr);

    ~AlgorithmTuner();

    void changeEvent(QEvent *e);


private:
    AlgorithmTunerSettings settings;
    GeneralSettings general_settings;

    std::vector<AlgorithmInterfacePtr> algorithms;
    std::vector<HVInterfacePtr> hv_algorithms;
    std::vector<EvaluatorInterfacePtr> evaluators;
    EvaluatorInterfacePtr current_evaluator;

    std::map<DatasetObjectPtr ,std::vector<std::pair<QWidget*,double>>> object_tuned_variables;

    ScapeDatasetPtr scapeDatasetPtr = nullptr;

    rapidcsv::CSVDocPtr data_info_doc;
    rapidcsv::CSVDocPtr data_doc;
    AlgorithmDataProc algorithmDataProc;
    int enabled_alg_variables_csv_start_col,n_enabled_alg_variables;
    std::map<std::string,int> enabled_alg_variable_indicies;

    vtkSmartPointer<vtkRenderWindow> render_window;
    vtkSmartPointer<vtkRenderer> renderer;
    pcl::shared_ptr<PointCloudGroupVisualizer> group_vis;
    QTimer *vis_timer_;

    QWidget* parameter_test_widget = nullptr;
    std::string parameter_test_name;

    void loadSettings();

    void saveSettings();

    void loadEvaluatorSettings();

    void saveEvaluatorSettings();

    void read_tuned_parameters();

    void update_tuned_parameters(DatasetObjectPtr &objPtr);

    void load_dataset();

    void add_results_to_visualizer(GeneticEvaluatorPtr &geneticEvaluatorPtr,std::string group,std::string node_prefix, std::vector<int> tp, std::vector<int> fp, std::vector<int> tn, std::vector<int> fn);

    void run_enabled_algorithms(GeneticEvaluatorPtr &geneticEvaluatorPtr,DatasetObjectPtr &obj, int &dpI);
private:
    EvaluatorInterfacePtr get_evaluator_interface(QString name);
    EvaluatorInterfacePtr get_evaluator_interface(std::string name);

    std::vector<double> get_param_test_values();

    void setSpinBoxWidgetValue(QWidget* widget, double val);

    double getSpinBoxWidgetValue(QWidget* widget);

    void set_doc_alg_var_names(rapidcsv::CSVDocPtr &csvDoc);

    void set_doc_alg_variables(rapidcsv::CSVDocPtr &csvDoc, int row_i);
private slots:

    void chose_dataset_folders();

    void update_evaluator(const int &i);

    void update_datapoint_spinbox(const QString &s);

    void update_ocs_and_gts(int i);

    void update_ocs_and_gts(const QString &s);

    void update_range_params(const QString &s);

    void update_range_param_limits(const QString &s);

    void run_enabled_algorithms();

    void load_tuned_parameters();

    void save_tuned_parameters();

    void save_data();

    void timeoutSlot();


};


#endif //MASTER_ALGORITHM_TUNER_HPP
