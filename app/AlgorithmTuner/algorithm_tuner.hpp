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
#include "ga_interface.hpp"
#include "ba_interface.hpp"
#include "genetic_evaluator_oc_interface.hpp"
#include "dataset/DatasetObject.hpp"
#include <dataset/scape/ScapeDataset.hpp>
#include "ga/visualization/point_cloud_group_visualizer.hpp"
#include "ga/genetic_evaluator/GeneticEvaluatorObjectCandidates.hpp"
#include "vtkSmartPointer.h"
#include "algorithm_data_proc.hpp"


struct AlgorithmTunerSettings {
    QString data_folder;
    QString recognition_folder;
    QString data_save_file;
};

struct GeneralSettings {
    QDoubleSpinBox *ground_truth_t_thresh = new QDoubleSpinBox();
    QDoubleSpinBox *ground_truth_r_thresh = new QDoubleSpinBox();
};

/*struct EvaluatorOCSettings {
    std::map<std::string, GeneticEvaluatorPtr> evaluator_map{{"GeneticEvaluatorOC", std::make_shared<GeneticEvaluatorOC>()}};

    std::vector<std::string> evaluator_types;

    QString current_evaluator_str;
    std::vector<QDoubleSpinBox *> currentDoubleSpinBoxes;
    std::vector<QSpinBox *> currentSpinBoxes;

    EvaluatorOCSettings() {
        for (auto &kvp:evaluator_map) {
            evaluator_types.push_back(kvp.first);
            evaluator_types_combo_box->addItem(QString::fromStdString(kvp.first));
        }
    }
};*/

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

    std::vector<HVInterfacePtr> hv_algorithms;
    std::vector<EvaluatorInterfacePtr> evaluators;
    QString current_evaluator_text;
    //EvaluatorOCSettings evaluator_settings;

    ScapeDatasetPtr scapeDatasetPtr = nullptr;

    AlgorithmDataProc algorithmDataProc;

    vtkSmartPointer<vtkRenderWindow> render_window;
    vtkSmartPointer<vtkRenderer> renderer;
    pcl::shared_ptr<PointCloudGroupVisualizer> group_vis;
    QTimer *vis_timer_;


    void loadSettings();

    void saveSettings();

    void loadEvaluatorSettings();

    void saveEvaluatorSettings();

    void load_dataset();

    void add_results_to_visualizer(GeneticEvaluatorPtr &geneticEvaluatorPtr,std::string group,std::string node_prefix, std::vector<int> &tp, std::vector<int> &fp, std::vector<int> &tn, std::vector<int> &fn);

    void run_enabled_algorithms(GeneticEvaluatorPtr &geneticEvaluatorPtr,DatasetObjectPtr &obj, DataPoint &dp, rawDataMapAlgObjVecT &rawDataMapAlgObjVec);
private:
    EvaluatorInterfacePtr get_evaluator_interface(QString name);
private slots:

    void chose_dataset_folders();

    void update_evaluator_type(const QString &s);

    void update_datapoint_spinbox(const QString &s);

    void update_ocs_and_gts(int i);

    void update_ocs_and_gts(const QString &s);

    void run_enabled_algorithms();

    void save_data();

    void bar_plot();

    void timeoutSlot();


};


#endif //MASTER_ALGORITHM_TUNER_HPP
