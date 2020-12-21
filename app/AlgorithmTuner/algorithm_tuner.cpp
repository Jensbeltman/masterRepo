#include "algorithm_tuner.hpp"
#include "ga/ga.hpp"
#include "baseline/baseline.hpp"
#include "dataset/transform_utility.hpp"
#include <dataset/scape/ScapeDataset.hpp>


//STL
#include <iostream>

// QT
#include <QEvent>
#include <QDialog>
#include <QObject>
#include <QFileDialog>
#include <QTableWidget>
#include <QDockWidget>

//VTK
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>
#include <vtkPointPicker.h>
//PCL
#include "pcl/common/transforms.h"

AlgorithmTuner::AlgorithmTuner(QMainWindow *parent) : QMainWindow(parent) {
    setupUi(this);
    QObject::connect(actionLoadDataset, &QAction::triggered, this, &AlgorithmTuner::chose_dataset_folders);
    QObject::connect(objectNameComboBox, &QComboBox::currentTextChanged, this,
                     &AlgorithmTuner::update_datapoint_spinbox);
    QObject::connect(objectNameComboBox, qOverload<const QString &>(&QComboBox::currentIndexChanged), this,
                     qOverload<const QString &>(&AlgorithmTuner::update_ocs_and_gts));
    QObject::connect(datapointSpinBox, qOverload<int>(&QSpinBox::valueChanged), this,
                     qOverload<int>(&AlgorithmTuner::update_ocs_and_gts));
    QObject::connect(runEnabledMethodsButton, &QPushButton::pressed, this, &AlgorithmTuner::run_enabled_methods);

    QObject::connect(evaluator_settings.evaluator_types_combo_box, &QComboBox::currentTextChanged, this,
                     &AlgorithmTuner::update_evaluator_type);




    //General Settings
    generalSettingsFormLayout->addRow(QString::fromStdString("ground_truth_t_thresh"),
                                      general_settings.ground_truth_t_thresh);
    generalSettingsFormLayout->addRow(QString::fromStdString("ground_truth_r_thresh"),
                                      general_settings.ground_truth_r_thresh);
    generalSettingsFormLayout->addRow(QString::fromStdString("icp_inlier_thresh"), general_settings.icp_inlier_thresh);
    //GA Settings
    gaSettingsFormLayout->addRow(QString::fromStdString("enable"), ga_settings.enable);
    gaSettingsFormLayout->addRow(QString::fromStdString("population_size"), ga_settings.population_size);
    gaSettingsFormLayout->addRow(QString::fromStdString("generation_max"), ga_settings.generation_max);
    gaSettingsFormLayout->addRow(QString::fromStdString("mutation_rate"), ga_settings.mutation_rate);
    gaSettingsFormLayout->addRow(QString::fromStdString("elite_pct"), ga_settings.elite_pct);
    gaSettingsFormLayout->addRow(QString::fromStdString("parent_pool_pct"), ga_settings.parent_pool_pct);
    //BA Settings
    baSettingsFormLayout->addRow(QString::fromStdString("enable"), ba_settings.enable);
    // Evaluator Settings
    evaluatorSettingsFormLayout->addRow(QString::fromStdString("Type"), evaluator_settings.evaluator_types_combo_box);
    update_evaluator_type(evaluator_settings.evaluator_types_combo_box->currentText());

    // Hide dataset and result dock as they will be reshown later
    datasetDock->hide();
    resultDock->hide();

    // Construction Visualizer
    render_window = vtkSmartPointer<vtkRenderWindow>::New();
    renderer = vtkSmartPointer<vtkRenderer>::New();
    render_window->AddRenderer(renderer);

    group_vis = pcl::make_shared<PointCloudGroupVisualizer>(renderer, render_window, "vis_src", false);

    this->setWindowTitle("Algorithm Tuner");

    // Set up the source window
    vtkWidget->SetRenderWindow(render_window);
    group_vis->setShowFPS(true);


    // Set up the source wi

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor_src = vtkWidget->GetInteractor();

    group_vis->setupInteractor(renderWindowInteractor_src, vtkWidget->GetRenderWindow());
    group_vis->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    group_vis->setShowFPS(false);
    vtkWidget->update();


    //Create a timer
    vis_timer_ = new QTimer(this);
    vis_timer_->start(5);//5ms

    connect(vis_timer_, SIGNAL (timeout()), this, SLOT (timeoutSlot()));


    loadSettings(); // Load setting from prev session e.g. paths for dataset
}


void AlgorithmTuner::changeEvent(QEvent *e) {
    QWidget::changeEvent(e);
    switch (e->type()) {
        case QEvent::LanguageChange:
            retranslateUi(this);
            break;
        default:
            break;
    }
}

void AlgorithmTuner::chose_dataset_folders() {
    QString data_folder = QFileDialog::getExistingDirectory(this, "Select data folder", settings.data_folder);
    QString recognition_folder = QFileDialog::getExistingDirectory(this, "Select recognition folder",
                                                                   settings.recognition_folder);

    if (!data_folder.isNull())
        settings.data_folder = data_folder;
    if (!recognition_folder.isNull())
        settings.recognition_folder = recognition_folder;

    std::cout << "data folder: " << data_folder.toStdString() << ", recognition folder: "
              << recognition_folder.toStdString() << std::endl;

    load_dataset();

}

void AlgorithmTuner::loadSettings() {
    QSettings qsettings;
    settings.data_folder = qsettings.value("data_folder", "/home").toString();
    settings.recognition_folder = qsettings.value("recognition_folder", "/home").toString();

    general_settings.ground_truth_t_thresh->setValue(
            qsettings.value("general_settings.ground_truth_t_thresh", "4").toDouble());
    general_settings.ground_truth_r_thresh->setValue(
            qsettings.value("general_settings.ground_truth_r_thresh", "5").toDouble());
    general_settings.icp_inlier_thresh->setValue(
            qsettings.value("general_settings.icp_inlier_thresh", "1.5").toDouble());
    ga_settings.enable->setChecked(qsettings.value("ga_settings.enable", "1").toBool());
    ga_settings.population_size->setValue(qsettings.value("ga_settings.population_size", "20").toInt());
    ga_settings.generation_max->setValue(qsettings.value("ga_settings.generation_max", "50").toInt());
    ga_settings.mutation_rate->setValue(qsettings.value("ga_settings.mutation_rate", "0.05").toDouble());
    ga_settings.elite_pct->setValue(qsettings.value("ga_settings.elite_pct", "0.1").toDouble());
    ga_settings.parent_pool_pct->setValue(qsettings.value("ga_settings.parent_pool_pct", "0.3").toDouble());

    ba_settings.enable->setChecked(qsettings.value("ba_settings.enable", "1").toBool());

    loadEvaluatorSettings();
}

void AlgorithmTuner::saveSettings() {
    QSettings qsettings;
    qsettings.setValue("data_folder", settings.data_folder);
    qsettings.setValue("recognition_folder", settings.recognition_folder);

    qsettings.setValue("general_settings.ground_truth_t_thresh", general_settings.ground_truth_t_thresh->value());
    qsettings.setValue("general_settings.ground_truth_r_thresh", general_settings.ground_truth_r_thresh->value());
    qsettings.setValue("general_settings.icp_inlier_thresh", general_settings.icp_inlier_thresh->value());

    qsettings.setValue("ga_settings.enable", ga_settings.enable->isChecked());
    qsettings.setValue("ga_settings.population_size", ga_settings.population_size->value());
    qsettings.setValue("ga_settings.generation_max", ga_settings.generation_max->value());
    qsettings.setValue("ga_settings.mutation_rate", ga_settings.mutation_rate->value());
    qsettings.setValue("ga_settings.elite_pct", ga_settings.elite_pct->value());
    qsettings.setValue("ga_settings.parent_pool_pct", ga_settings.parent_pool_pct->value());

    qsettings.setValue("ba_settings.enable", ba_settings.enable->isChecked());

    saveEvaluatorSettings();
}

AlgorithmTuner::~AlgorithmTuner() {
    saveSettings();
    delete this;
}


void AlgorithmTuner::load_dataset() {
    scapeDatasetPtr = std::make_shared<ScapeDataset>(settings.data_folder.toStdString(),
                                                     settings.recognition_folder.toStdString(), true);

    objectNameComboBox->clear();
    for (auto &sobj:scapeDatasetPtr->objects)
        objectNameComboBox->addItem(QString::fromStdString(sobj->name));
    datapointSpinBox->setValue(0);
    datasetDock->show();
}

void AlgorithmTuner::update_datapoint_spinbox(const QString &s) {
    std::string stds = s.toStdString();
    if(stds !="")
        datapointSpinBox->setMaximum(scapeDatasetPtr->get_scape_object_by_name(stds)->data_points.size());
}


void AlgorithmTuner::update_ocs_and_gts(int i) {
    auto &dp = scapeDatasetPtr->get_scape_object_by_name(
            objectNameComboBox->currentText().toStdString())->data_points[i];
    ocsCountLabel->setText(QString::fromStdString(std::to_string(dp.ocs.size())));
    gtsCountLabel->setText(QString::fromStdString(std::to_string(dp.gts.size())));
}

void AlgorithmTuner::update_ocs_and_gts(const QString &s) {

    auto sob = scapeDatasetPtr->get_scape_object_by_name(s.toStdString());
    DataPoint *dp;
    if(sob != nullptr) {
        if (sob->data_points.size() && datapointSpinBox->value() < sob->data_points.size())
            dp = &sob->data_points[datapointSpinBox->value()];
        else if (sob->data_points.size())
            dp = &sob->data_points[0];

        ocsCountLabel->setText(QString::fromStdString(std::to_string(dp->ocs.size())));
        gtsCountLabel->setText(QString::fromStdString(std::to_string(dp->gts.size())));
    }
}


void getFPTN(int &tp, int &fp, int &tn, int &fn, chromosomeT chromosome, chromosomeT correct_ocs) {
    for (int i = 0; i < correct_ocs.size(); i++) {
        if (correct_ocs[i]) {
            if (chromosome[i]) {
                tp++;
            } else {
                fn++;
            }
        } else {
            if (chromosome[i]) {
                fp++;
            } else {
                tn++;
            }
        }
    }
}

void clearLayout(QLayout *layout) {
    QLayoutItem *item;
    while ((item = layout->takeAt(0))) {
        if (item->layout()) {
            clearLayout(item->layout());
            delete item->layout();
        }
        if (item->widget()) {
            delete item->widget();
        }
        delete item;
    }
}

void AlgorithmTuner::run_enabled_methods() {
    DatasetObjectPtr ob = scapeDatasetPtr->get_object_by_name(objectNameComboBox->currentText().toStdString());
    GeneticEvaluatorOCPtr &geneticEvaluatorOCPtr = evaluator_settings.evaluator_map[evaluator_settings.evaluator_types_combo_box->currentText().toStdString()];

    std::vector<double> evaluator_hyper_params_d;
    for(auto& sp:evaluator_settings.currentDoubleSpinBoxes)
        evaluator_hyper_params_d.emplace_back(sp->value());

    std::vector<int> evaluator_hyper_params_i;
    for(auto& sp:evaluator_settings.currentSpinBoxes)
        evaluator_hyper_params_i.emplace_back(sp->value());

    geneticEvaluatorOCPtr->setHyperParameters_d(evaluator_hyper_params_d);
    geneticEvaluatorOCPtr->setHyperParameters_i(evaluator_hyper_params_i);



    geneticEvaluatorOCPtr->initialise_object(ob, datapointSpinBox->value());
    std::make_shared<GeneticEvaluatorOC>(ob, datapointSpinBox->value(), general_settings.icp_inlier_thresh->value());
    auto &dp = ob->data_points[datapointSpinBox->value()];

    TransformUtility tu;
    std::vector<bool> correct_ocs = tu.get_true_ocs(dp.ocs, dp.gts, general_settings.ground_truth_t_thresh->value(),
                                                    general_settings.ground_truth_r_thresh->value());


    GAResult gaResult;
    if (ga_settings.enable->isChecked()) {
        GA ga(dp.ocs.size(), ga_settings.population_size->value(), ga_settings.generation_max->value(),
              ga_settings.mutation_rate->value(), ga_settings.elite_pct->value(), ga_settings.parent_pool_pct->value());
        ga.geneticEvaluatorPtr = std::dynamic_pointer_cast<GeneticEvaluator>(geneticEvaluatorOCPtr);
        gaResult = ga.solve();
    }

    BAResult baResult;
    if (ga_settings.enable->isChecked()) {
        Baseline baseline(geneticEvaluatorOCPtr);
        baResult = baseline.solve();
    }

    int ga_tp = 0, ga_fp = 0, ga_tn = 0, ga_fn = 0, ba_tp = 0, ba_fp = 0, ba_tn = 0, ba_fn = 0;
    getFPTN(ga_tp, ga_fp, ga_tn, ga_fn, gaResult.chromosome, correct_ocs);
    getFPTN(ba_tp, ba_fp, ba_tn, ba_fn, baResult.chromosome, correct_ocs);

    //result dock
    while (resultDockFormLayout->count() > 0)
        resultDockFormLayout->removeRow(0);

    resultDockFormLayout->addRow(QString::fromStdString("ga_tp"),
                                 new QLabel(QString::fromStdString(std::to_string(ga_tp))));
    resultDockFormLayout->addRow(QString::fromStdString("ga_fp"),
                                 new QLabel(QString::fromStdString(std::to_string(ga_fp))));
    resultDockFormLayout->addRow(QString::fromStdString("ga_tn"),
                                 new QLabel(QString::fromStdString(std::to_string(ga_tn))));
    resultDockFormLayout->addRow(QString::fromStdString("ga_fn"),
                                 new QLabel(QString::fromStdString(std::to_string(ga_fn))));
    resultDockFormLayout->addRow(QString::fromStdString("ba_tp"),
                                 new QLabel(QString::fromStdString(std::to_string(ba_tp))));
    resultDockFormLayout->addRow(QString::fromStdString("ba_fp"),
                                 new QLabel(QString::fromStdString(std::to_string(ba_fp))));
    resultDockFormLayout->addRow(QString::fromStdString("ba_tn"),
                                 new QLabel(QString::fromStdString(std::to_string(ba_tn))));
    resultDockFormLayout->addRow(QString::fromStdString("ba_fn"),
                                 new QLabel(QString::fromStdString(std::to_string(ba_fn))));
    resultDock->show();

    // VISUALISATION
    group_vis->clear();
    group_vis->addIdPointCloud(geneticEvaluatorOCPtr->pc, "Captured Point Cloud");
    PointCloudT::Ptr mesh_pc = geneticEvaluatorOCPtr->pcm;

    for (int i = 0; i < geneticEvaluatorOCPtr->dp.ocs.size(); i++) {
        std::string id = "oc_" + std::to_string(i);

        if (gaResult.chromosome[i] && baResult.chromosome[i]) {
            group_vis->addIdPointCloud(geneticEvaluatorOCPtr->visible_oc_pcs[i], id, "Accepted by both", 0, 255, 0);
        } else if ((!gaResult.chromosome[i]) && (!baResult.chromosome[i])) {
            group_vis->addIdPointCloud(geneticEvaluatorOCPtr->visible_oc_pcs[i], id, "Rejected by both", 255, 0, 0);
        } else if ((gaResult.chromosome[i]) && (!baResult.chromosome[i])) {
            group_vis->addIdPointCloud(geneticEvaluatorOCPtr->visible_oc_pcs[i], id, "Rejcted by baseline", 255, 255,
                                       0);
        } else if ((!gaResult.chromosome[i]) && (baResult.chromosome[i])) {
            group_vis->addIdPointCloud(geneticEvaluatorOCPtr->visible_oc_pcs[i], id, "Accepted by baseline", 0, 0, 255);
        }
    }

    // Add gt to visualization
    PointCloudT::Ptr meshpc = ob->get_mesh_point_cloud();
    for (int g = 0; g < dp.gts.size(); g++) {
        PointCloudT::Ptr gtpc_vis = pcl::make_shared<PointCloudT>();
        std::string id = "gt_" + std::to_string(g);
        pcl::transformPointCloud(*meshpc, *gtpc_vis, dp.gts[g]);
        group_vis->addIdPointCloud(gtpc_vis, id, "Ground Truth", 0, 255, 255);
    }
    group_vis->resetCamera();
    group_vis->updateSelector();

}

void AlgorithmTuner::timeoutSlot() {
    vtkWidget->update();
}

void AlgorithmTuner::update_evaluator_type(const QString &s) {
    saveEvaluatorSettings();
    evaluator_settings.current_evaluator_str = s;
    loadEvaluatorSettings();
}

void AlgorithmTuner::loadEvaluatorSettings() {
    // Remove all setting form entries besides the combobox(first)
    for (auto &sb:evaluator_settings.currentDoubleSpinBoxes) {
        evaluatorSettingsFormLayout->labelForField(sb)->deleteLater();
        sb->deleteLater();
    }
    for (auto &sb:evaluator_settings.currentSpinBoxes) {
        evaluatorSettingsFormLayout->labelForField(sb)->deleteLater();
        sb->deleteLater();
    }
    //Clear the vector keeping track of setting spin boxes
    evaluator_settings.currentDoubleSpinBoxes.clear();
    evaluator_settings.currentSpinBoxes.clear();

    // Load hyper parameter names and values
    std::vector<std::string> hyper_param_names_d;
    std::vector<double *> hyper_params_d;
    std::vector<std::string> hyper_param_names_i;
    std::vector<int *> hyper_params_i;
    evaluator_settings.evaluator_map[evaluator_settings.current_evaluator_str.toStdString()]->getHyperParameters_d(
            hyper_param_names_d, hyper_params_d);
    evaluator_settings.evaluator_map[evaluator_settings.current_evaluator_str.toStdString()]->getHyperParameters_i(
            hyper_param_names_i, hyper_params_i);


    // Load hyper parameter from qsetting and create new spinboxes and form entries for each. Default value is taken from evaluator type defaults.
    QSettings qsettings;
    for (int i = 0; i < hyper_param_names_d.size(); i++) {
        auto &param_name = hyper_param_names_d[i];
        auto &param_value_ptr = hyper_params_d[i];
        QDoubleSpinBox *qDoubleSpinBox = new QDoubleSpinBox;
        qDoubleSpinBox->setValue(
                qsettings.value(evaluator_settings.current_evaluator_str + "/" + QString::fromStdString(param_name),*param_value_ptr).toDouble());
        evaluator_settings.currentDoubleSpinBoxes.push_back(qDoubleSpinBox);
        evaluatorSettingsFormLayout->addRow(QString::fromStdString(param_name), qDoubleSpinBox);
    }

    for (int i = 0; i < hyper_param_names_i.size(); i++) {
        auto &param_name = hyper_param_names_i[i];
        auto &param_value_ptr = hyper_params_i[i];
        QSpinBox *qSpinBox = new QSpinBox;
        qSpinBox->setValue(
                qsettings.value(evaluator_settings.current_evaluator_str + "/" + QString::fromStdString(param_name),*param_value_ptr).toInt());
        evaluator_settings.currentSpinBoxes.push_back(qSpinBox);
        evaluatorSettingsFormLayout->addRow(QString::fromStdString(param_name), qSpinBox);
    }
}

void AlgorithmTuner::saveEvaluatorSettings() {
    QSettings qsettings;
    for (auto &dsb:evaluator_settings.currentDoubleSpinBoxes)
        qsettings.setValue(evaluator_settings.current_evaluator_str + "/" +qobject_cast<QLabel *>(evaluatorSettingsFormLayout->labelForField(dsb))->text(),
                           dsb->value());

    for (auto &sb:evaluator_settings.currentSpinBoxes)
        qsettings.setValue(evaluator_settings.current_evaluator_str + "/" +qobject_cast<QLabel *>(evaluatorSettingsFormLayout->labelForField(sb))->text(), sb->value());

}