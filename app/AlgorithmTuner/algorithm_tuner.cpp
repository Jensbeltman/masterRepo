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
    QObject::connect(runEnabledOnDatasetButton, &QPushButton::pressed, this,
                     &AlgorithmTuner::run_enabled_methods_on_dataset);

    QObject::connect(evaluator_settings.evaluator_types_combo_box, &QComboBox::currentTextChanged, this,
                     &AlgorithmTuner::update_evaluator_type);

    algorithms.push_back(std::make_shared<GAInterface>());
    algorithms.push_back(std::make_shared<BAInterface>());

    for(auto &alg:algorithms){
        QWidget* widget = new QTabWidget();
        QFormLayout* qFormLayout = new QFormLayout();

        for(auto &var:alg->variables_b)
            qFormLayout->addRow(new QLabel(QString::fromStdString(var.name)),var.checkBox);
        for(auto &var:alg->variables_i)
            qFormLayout->addRow(new QLabel(QString::fromStdString(var.name)),var.spinBox);
        for(auto &var:alg->variables_d)
            qFormLayout->addRow(new QLabel(QString::fromStdString(var.name)),var.spinBox);

        widget->setLayout(qFormLayout);
        settingsTabWidget->addTab(widget,QString::fromStdString(alg->name));
    }

    //General Settings
    generalSettingsFormLayout->addRow(QString::fromStdString("ground_truth_t_thresh"),
                                      general_settings.ground_truth_t_thresh);
    generalSettingsFormLayout->addRow(QString::fromStdString("ground_truth_r_thresh"),
                                      general_settings.ground_truth_r_thresh);

    // Evaluator Settings
    evaluatorSettingsFormLayout->addRow(QString::fromStdString("Type"), evaluator_settings.evaluator_types_combo_box);
    update_evaluator_type(evaluator_settings.evaluator_types_combo_box->currentText());


    // Hide dataset and result dock as they will be reshown later
    datasetDock->hide();
    resultDock->hide();
    datasetResultDock->hide();

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

    connect(vis_timer_, SIGNAL(timeout()), this, SLOT(timeoutSlot()));


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

    for(auto &alg:algorithms)
        alg->load_settings(qsettings);

    loadEvaluatorSettings();
}

void AlgorithmTuner::saveSettings() {
    QSettings qsettings;
    qsettings.setValue("data_folder", settings.data_folder);
    qsettings.setValue("recognition_folder", settings.recognition_folder);

    qsettings.setValue("general_settings.ground_truth_t_thresh", general_settings.ground_truth_t_thresh->value());
    qsettings.setValue("general_settings.ground_truth_r_thresh", general_settings.ground_truth_r_thresh->value());

    for(auto &alg:algorithms)
        alg->save_settings(qsettings);

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
    if (stds != "")
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
    if (sob != nullptr) {
        if (sob->data_points.size() && datapointSpinBox->value() < sob->data_points.size())
            dp = &sob->data_points[datapointSpinBox->value()];
        else if (sob->data_points.size())
            dp = &sob->data_points[0];

        ocsCountLabel->setText(QString::fromStdString(std::to_string(dp->ocs.size())));
        gtsCountLabel->setText(QString::fromStdString(std::to_string(dp->gts.size())));
    }
}


void
getFPTN(std::vector<int> &tp, std::vector<int> &fp, std::vector<int> &tn, std::vector<int> &fn, chromosomeT chromosome,
        chromosomeT correct_ocs) {
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
    QPushButton *button = (QPushButton *)sender();
    if (scapeDatasetPtr != nullptr) {
        DatasetObjectPtr ob = scapeDatasetPtr->get_object_by_name(objectNameComboBox->currentText().toStdString());
        GeneticEvaluatorOCPtr &geneticEvaluatorOCPtr = evaluator_settings.evaluator_map[evaluator_settings.evaluator_types_combo_box->currentText().toStdString()];

        std::vector<double> evaluator_hyper_params_d;
        for (auto &sp:evaluator_settings.currentDoubleSpinBoxes)
            evaluator_hyper_params_d.emplace_back(sp->value());

        std::vector<int> evaluator_hyper_params_i;
        for (auto &sp:evaluator_settings.currentSpinBoxes)
            evaluator_hyper_params_i.emplace_back(sp->value());

        geneticEvaluatorOCPtr->setHyperParameters_d(evaluator_hyper_params_d);
        geneticEvaluatorOCPtr->setHyperParameters_i(evaluator_hyper_params_i);


        geneticEvaluatorOCPtr->initialise_object(ob, datapointSpinBox->value());

        auto &dp = ob->data_points[datapointSpinBox->value()];

        TransformUtility tu;
        std::vector<bool> correct_ocs = tu.get_true_ocs(dp.ocs, dp.gts, general_settings.ground_truth_t_thresh->value(),
                                                        general_settings.ground_truth_r_thresh->value());

        // Clear old results
        while (resultDockFormLayout->count() > 0)
            resultDockFormLayout->removeRow(0);

        // Clear Visualizer
        group_vis->clear();
        group_vis->addIdPointCloud(geneticEvaluatorOCPtr->pc, "Captured Point Cloud");
        PointCloudT::Ptr mesh_pc = geneticEvaluatorOCPtr->pcm;

        // Add gt and data to visualization
        PointCloudT::Ptr meshpc = ob->get_mesh_point_cloud();
        for (int g = 0; g < dp.gts.size(); g++) {
            PointCloudT::Ptr gtpc_vis = pcl::make_shared<PointCloudT>();
            std::string id = "gt_" + std::to_string(g);
            pcl::transformPointCloud(*meshpc, *gtpc_vis, dp.gts[g]);
            group_vis->addIdPointCloud(gtpc_vis, id, "Ground Truth", 0, 255, 255);
        }
        group_vis->resetCamera();
        group_vis->update_text();


        for(auto &alg:algorithms){
            if(alg->enabled()) {
                alg->update_variables();
                std::vector<int> tp, fp, tn, fn;
                alg->run(geneticEvaluatorOCPtr, correct_ocs, tp, fp, tn, fn);
                int n_tp = tp.size(), n_fp = fp.size(), n_tn = tn.size(), n_fn = fn.size();

                resultDockFormLayout->addRow(QString::fromStdString(alg->name + " accuracy"),
                                             new QLabel(QString::fromStdString(std::to_string(
                                                     static_cast<double>(n_tp + n_tn) /
                                                     static_cast<double>(n_tp + n_fp + n_tn + n_fn)))));
                resultDockFormLayout->addRow(QString::fromStdString(alg->name + "(tp)"),
                                             new QLabel(QString::fromStdString(std::to_string(n_tp))));
                resultDockFormLayout->addRow(QString::fromStdString(alg->name + "(fp)"),
                                             new QLabel(QString::fromStdString(std::to_string(n_fp))));
                resultDockFormLayout->addRow(QString::fromStdString(alg->name + "(tn)"),
                                             new QLabel(QString::fromStdString(std::to_string(n_tn))));
                resultDockFormLayout->addRow(QString::fromStdString(alg->name + "(fn)"),
                                             new QLabel(QString::fromStdString(std::to_string(n_fn))));

                std::string prefix = alg->name;
                std::for_each(prefix.begin(), prefix.end(), [](char &c) { c = std::tolower(c); });

                add_results_to_visualizer(geneticEvaluatorOCPtr, alg->name, prefix, tp, fp, tn, fn);
            }
        }
        resultDock->show();
    }
}

void AlgorithmTuner::run_enabled_methods_on_dataset() {
    // Clear Visualizer
    group_vis->clear();

    if (scapeDatasetPtr != nullptr) {
        while (datasetResultformLayout->count() > 0)
            datasetResultformLayout->removeRow(0);

        std::map<std::string, std::vector<std::array<std::vector<int>, 4>>> hist_tfpn;
        std::map<std::string, std::array<std::vector<int>, 4>> hist_n_tfpn;
        std::map<std::string, std::array<std::vector<double>, 4>> hist_pct_tfpn;
        std::map<std::string, std::vector<double>> hist_accuracy;
        for (auto &alg:algorithms) {
            hist_n_tfpn.emplace(std::make_pair(alg->name, std::array<std::vector<int>, 4>()));
            hist_tfpn.emplace(std::make_pair(alg->name, std::vector<std::array<std::vector<int>, 4>>()));
            hist_pct_tfpn.emplace(std::make_pair(alg->name,  std::array<std::vector<double>,4>()));
            hist_accuracy.emplace(std::make_pair(alg->name, std::vector<double>()));
        }

        TransformUtility tu;
        GeneticEvaluatorOCPtr &geneticEvaluatorOCPtr = evaluator_settings.evaluator_map[evaluator_settings.evaluator_types_combo_box->currentText().toStdString()];

        std::vector<double> evaluator_hyper_params_d;
        for (auto &sp:evaluator_settings.currentDoubleSpinBoxes) {
            evaluator_hyper_params_d.emplace_back(sp->value());
        }
        std::vector<int> evaluator_hyper_params_i;
        for (auto &sp:evaluator_settings.currentSpinBoxes) {
            evaluator_hyper_params_i.emplace_back(sp->value());
        }
        geneticEvaluatorOCPtr->setHyperParameters_d(evaluator_hyper_params_d);
        geneticEvaluatorOCPtr->setHyperParameters_i(evaluator_hyper_params_i);


        for (auto &obj:scapeDatasetPtr->objects) {
            int ndp = obj->data_points.size();
            int cndp = 0;
            for (auto &dp:obj->data_points) {
                if(!dp.gts.empty()) {
                    std::cout << "\n Object: " << std::setw(12) << obj->name << ", Datapoint: " << std::setw(3)
                              << ++cndp << "/" << std::setw(3) << ndp << ", Filename: " << dp.pcd_filename << std::endl;

                    geneticEvaluatorOCPtr->initialise_object(obj, datapointSpinBox->value());

                    std::vector<bool> correct_ocs = tu.get_true_ocs(dp.ocs, dp.gts,
                                                                    general_settings.ground_truth_t_thresh->value(),
                                                                    general_settings.ground_truth_r_thresh->value());
                    // Clear old results
                    while (resultDockFormLayout->count() > 0)
                        resultDockFormLayout->removeRow(0);


                    for (auto &alg:algorithms) {
                        alg->update_variables();
                        if (alg->enabled()) {
                            std::vector<int> tp, fp, tn, fn;
                            alg->run(geneticEvaluatorOCPtr, correct_ocs, tp, fp, tn, fn);
                            int n_tp = tp.size(), n_fp = fp.size(), n_tn = tn.size(), n_fn = fn.size();

                            hist_tfpn[alg->name].emplace_back(std::array<std::vector<int>, 4>{tp, fp, tn, fn});
                            hist_n_tfpn[alg->name][0].emplace_back(n_tp);
                            hist_n_tfpn[alg->name][1].emplace_back(n_fp);
                            hist_n_tfpn[alg->name][2].emplace_back(n_tn);
                            hist_n_tfpn[alg->name][3].emplace_back(n_fn);
                            hist_pct_tfpn[alg->name][0].emplace_back(static_cast<double>(n_tp)/dp.ocs.size());
                            hist_pct_tfpn[alg->name][1].emplace_back(static_cast<double>(n_fp)/dp.ocs.size());
                            hist_pct_tfpn[alg->name][2].emplace_back(static_cast<double>(n_tn)/dp.ocs.size());
                            hist_pct_tfpn[alg->name][3].emplace_back(static_cast<double>(n_fn)/dp.ocs.size());

                            hist_accuracy[alg->name].emplace_back(
                                    static_cast<double>(n_tp + n_tn) / static_cast<double>(n_tp + n_fp + n_tn + n_fn));
                        }
                    }
                }
            }
        }

        for (auto &alg:algorithms) {
            if (alg->enabled()) {
                datasetResultformLayout->addRow(QString::fromStdString(alg->name + " acc avr"),
                                                new QLabel(QString::fromStdString(std::to_string(
                                                        std::accumulate(hist_accuracy[alg->name].begin(), hist_accuracy[alg->name].end(), 0.0) /
                                                        static_cast<double>(hist_accuracy[alg->name].size())))));
                datasetResultformLayout->addRow(QString::fromStdString(alg->name + " tp avr"),
                                                new QLabel(QString::fromStdString(std::to_string(
                                                        std::accumulate(hist_pct_tfpn[alg->name][0].begin(), hist_pct_tfpn[alg->name][0].end(), 0.0) /
                                                        static_cast<double>(hist_pct_tfpn[alg->name][0].size())))));
                datasetResultformLayout->addRow(QString::fromStdString(alg->name + " fp avr"),
                                                new QLabel(QString::fromStdString(std::to_string(
                                                        std::accumulate(hist_pct_tfpn[alg->name][1].begin(), hist_pct_tfpn[alg->name][1].end(), 0.0) /
                                                        static_cast<double>(hist_pct_tfpn[alg->name][1].size())))));
                datasetResultformLayout->addRow(QString::fromStdString(alg->name + " tn avr"),
                                                new QLabel(QString::fromStdString(std::to_string(
                                                        std::accumulate(hist_pct_tfpn[alg->name][2].begin(), hist_pct_tfpn[alg->name][2].end(), 0.0) /
                                                        static_cast<double>(hist_pct_tfpn[alg->name][2].size())))));
                datasetResultformLayout->addRow(QString::fromStdString(alg->name + " fn avr"),
                                                new QLabel(QString::fromStdString(std::to_string(
                                                        std::accumulate(hist_pct_tfpn[alg->name][3].begin(), hist_pct_tfpn[alg->name][3].end(), 0.0) /
                                                        static_cast<double>(hist_pct_tfpn[alg->name][3].size())))));
            }
        }
        datasetResultDock->show();
    }
}

void AlgorithmTuner::add_results_to_visualizer(GeneticEvaluatorOCPtr &geneticEvaluatorOcPtr, std::string group,
                                               std::string node_prefix, std::vector<int> &tp, std::vector<int> &fp,
                                               std::vector<int> &tn, std::vector<int> &fn) {
    for (int i = 0; i < geneticEvaluatorOcPtr->dp.ocs.size(); i++) {
        std::string id = node_prefix + "_oc_" + std::to_string(i);

        if (std::find(tp.begin(), tp.end(), i) != tp.end()) {
            group_vis->addIdPointCloud(geneticEvaluatorOcPtr->visible_oc_pcs[i], id,
                                       group + "/True Positives", 0, 255, 0);
        } else if (std::find(fp.begin(), fp.end(), i) != fp.end()) {
            group_vis->addIdPointCloud(geneticEvaluatorOcPtr->visible_oc_pcs[i], id,
                                       group + "/False Positives", 255, 0, 0);
        } else if (std::find(tn.begin(), tn.end(), i) != tn.end()) {
            group_vis->addIdPointCloud(geneticEvaluatorOcPtr->visible_oc_pcs[i], id,
                                       group + "/True Negatives", 255, 255, 0);
        } else if (std::find(fn.begin(), fn.end(), i) != fn.end()) {
            group_vis->addIdPointCloud(geneticEvaluatorOcPtr->visible_oc_pcs[i], id,
                                       group + "/False Negatives", 0, 0, 255);
        }
    }
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
                qsettings.value(evaluator_settings.current_evaluator_str + "/" + QString::fromStdString(param_name),
                                *param_value_ptr).toDouble());
        evaluator_settings.currentDoubleSpinBoxes.push_back(qDoubleSpinBox);
        evaluatorSettingsFormLayout->addRow(QString::fromStdString(param_name), qDoubleSpinBox);
    }

    for (int i = 0; i < hyper_param_names_i.size(); i++) {
        auto &param_name = hyper_param_names_i[i];
        auto &param_value_ptr = hyper_params_i[i];
        QSpinBox *qSpinBox = new QSpinBox;
        qSpinBox->setValue(
                qsettings.value(evaluator_settings.current_evaluator_str + "/" + QString::fromStdString(param_name),
                                *param_value_ptr).toInt());
        evaluator_settings.currentSpinBoxes.push_back(qSpinBox);
        evaluatorSettingsFormLayout->addRow(QString::fromStdString(param_name), qSpinBox);
    }
}

void AlgorithmTuner::saveEvaluatorSettings() {
    QSettings qsettings;
    for (auto &dsb:evaluator_settings.currentDoubleSpinBoxes)
        qsettings.setValue(evaluator_settings.current_evaluator_str + "/" +
                           qobject_cast<QLabel *>(evaluatorSettingsFormLayout->labelForField(dsb))->text(),
                           dsb->value());

    for (auto &sb:evaluator_settings.currentSpinBoxes)
        qsettings.setValue(evaluator_settings.current_evaluator_str + "/" +
                           qobject_cast<QLabel *>(evaluatorSettingsFormLayout->labelForField(sb))->text(), sb->value());

}


