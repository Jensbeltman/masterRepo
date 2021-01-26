#include "algorithm_tuner.hpp"
#include "ga/ga.hpp"
#include "baseline/baseline.hpp"
#include "dataset/transform_utility.hpp"
#include <dataset/scape/ScapeDataset.hpp>


//STL
#include <iostream>

//DATAUTIL
#include "datautil/timeinfo.hpp"

// QT
#include <QEvent>
#include <QDialog>
#include <QObject>
#include <QFileDialog>
#include <QTableWidget>
#include <QDockWidget>
#include <QEventLoop>

//VTK
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>
#include <vtkPointPicker.h>
//PCL
#include "pcl/common/transforms.h"

AlgorithmTuner::AlgorithmTuner(QMainWindow *parent) : QMainWindow(parent) {
    setupUi(this);
    this->setWindowTitle("Algorithm Tuner");
    QObject::connect(actionLoadDataset, &QAction::triggered, this, &AlgorithmTuner::chose_dataset_folders);
    QObject::connect(objectNameComboBox, &QComboBox::currentTextChanged, this,
                     &AlgorithmTuner::update_datapoint_spinbox);
    QObject::connect(objectNameComboBox, qOverload<const QString &>(&QComboBox::currentIndexChanged), this,
                     qOverload<const QString &>(&AlgorithmTuner::update_ocs_and_gts));
    QObject::connect(datapointSpinBox, qOverload<int>(&QSpinBox::valueChanged), this,
                     qOverload<int>(&AlgorithmTuner::update_ocs_and_gts));
    QObject::connect(runEnabledMethodsButton, &QPushButton::pressed, this,
                     qOverload<>(&AlgorithmTuner::run_enabled_algorithms));
    QObject::connect(runEnabledOnObjectButton, &QPushButton::pressed, this,
                     qOverload<>(&AlgorithmTuner::run_enabled_algorithms));
    QObject::connect(runEnabledOnDatasetButton, &QPushButton::pressed, this,
                     qOverload<>(&AlgorithmTuner::run_enabled_algorithms));
    QObject::connect(dataProcSaveResultsButton, &QPushButton::pressed, this, &AlgorithmTuner::save_data);
    QObject::connect(barPlotButton, &QPushButton::pressed, this, &AlgorithmTuner::bar_plot);
    QObject::connect(evaluator_settings.evaluator_types_combo_box, &QComboBox::currentTextChanged, this,
                     &AlgorithmTuner::update_evaluator_type);

    // Disable button that arent currently usable
    runEnabledMethodsButton->setEnabled(false);
    runEnabledOnObjectButton->setEnabled(false);
    runEnabledOnDatasetButton->setEnabled(false);
    dataProcSaveResultsButton->setEnabled(false);

    // Hide docks and widgets
    datasetDock->hide();
    resultDock->hide();
    dataProcDock->hide();
    progressBar->hide();

    // Create algorithms interfaces and add variables to GUI
    algorithms.push_back(std::make_shared<GAInterface>());
    algorithms.push_back(std::make_shared<BAInterface>());

    for (auto &alg:algorithms) {
        QWidget *widget = new QTabWidget();
        QFormLayout *qFormLayout = new QFormLayout();

        for (auto &var:alg->variables_b)
            qFormLayout->addRow(new QLabel(QString::fromStdString(var.name)), var.checkBox);
        for (auto &var:alg->variables_i)
            qFormLayout->addRow(new QLabel(QString::fromStdString(var.name)), var.spinBox);
        for (auto &var:alg->variables_d)
            qFormLayout->addRow(new QLabel(QString::fromStdString(var.name)), var.spinBox);

        widget->setLayout(qFormLayout);
        settingsTabWidget->addTab(widget, QString::fromStdString(alg->name));
    }

    //Add general Settings to GUI
    generalSettingsFormLayout->addRow(QString::fromStdString("ground_truth_t_thresh"),
                                      general_settings.ground_truth_t_thresh);
    generalSettingsFormLayout->addRow(QString::fromStdString("ground_truth_r_thresh"),
                                      general_settings.ground_truth_r_thresh);

    //Add Evaluator Settings to GUI
    evaluatorSettingsFormLayout->addRow(QString::fromStdString("Type"), evaluator_settings.evaluator_types_combo_box);
    update_evaluator_type(evaluator_settings.evaluator_types_combo_box->currentText());


    // Construction Visualizer
    render_window = vtkSmartPointer<vtkRenderWindow>::New();
    renderer = vtkSmartPointer<vtkRenderer>::New();
    render_window->AddRenderer(renderer);

    group_vis = pcl::make_shared<PointCloudGroupVisualizer>(renderer, render_window, "vis_src", false);

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


void AlgorithmTuner::load_dataset() {
    scapeDatasetPtr = std::make_shared<ScapeDataset>(settings.data_folder.toStdString(),
                                                     settings.recognition_folder.toStdString(), true);

    objectNameComboBox->clear();
    for (auto &sobj:scapeDatasetPtr->objects)
        objectNameComboBox->addItem(QString::fromStdString(sobj->name));
    datapointSpinBox->setValue(0);
    datasetDock->show();
    runEnabledMethodsButton->setEnabled(true);
    runEnabledOnObjectButton->setEnabled(true);
    runEnabledOnDatasetButton->setEnabled(true);
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


void AlgorithmTuner::run_enabled_algorithms(GeneticEvaluatorOCPtr &geneticEvaluatorOcPtr, DatasetObjectPtr &obj,
                                            DataPoint &dp, rawDataMapAlgObjVecT &rawDataMapAlgObjVec) {
    if (geneticEvaluatorOcPtr->datasetObjectPtr == nullptr) {
        geneticEvaluatorOcPtr->initialise_object(obj, dp);
    } else if (geneticEvaluatorOcPtr->datasetObjectPtr->name != obj->name) {
        geneticEvaluatorOcPtr->initialise_object(obj, dp);
    } else if ((geneticEvaluatorOcPtr->dp.oc_scores != dp.oc_scores) ||
               (geneticEvaluatorOcPtr->dp.ground_truth_path != dp.ground_truth_path)) {
        geneticEvaluatorOcPtr->initialise_datapoint(dp);
    }

    for (auto &alg:algorithms) {
        alg->update_variables();
        if (alg->enabled()) {
            rawDataMapAlgObjVec[alg->name][obj].emplace_back(alg->run(geneticEvaluatorOcPtr));
        }
    }
}


void AlgorithmTuner::run_enabled_algorithms() {
    progressBar->show();
    if (scapeDatasetPtr != nullptr) {
        QPushButton *button = (QPushButton *) sender();

        GeneticEvaluatorOCPtr &geneticEvaluatorOCPtr = evaluator_settings.evaluator_map[evaluator_settings.evaluator_types_combo_box->currentText().toStdString()];

        std::vector<double> evaluator_hyper_params_d;
        for (auto &sp:evaluator_settings.currentDoubleSpinBoxes)
            evaluator_hyper_params_d.emplace_back(sp->value());

        std::vector<int> evaluator_hyper_params_i;
        for (auto &sp:evaluator_settings.currentSpinBoxes)
            evaluator_hyper_params_i.emplace_back(sp->value());

        geneticEvaluatorOCPtr->setHyperParameters_d(evaluator_hyper_params_d);
        geneticEvaluatorOCPtr->setHyperParameters_i(evaluator_hyper_params_i);

        rawDataMapAlgObjVecT rawDataMapAlgObjVec;

        int n_dp = 0, tot_n_dp = 0;

        if (button == runEnabledMethodsButton) {
            DatasetObjectPtr obj = scapeDatasetPtr->get_object_by_name(objectNameComboBox->currentText().toStdString());
            auto &dp = obj->data_points[datapointSpinBox->value()];

            run_enabled_algorithms(geneticEvaluatorOCPtr, obj, dp, rawDataMapAlgObjVec);
        } else if (button == runEnabledOnObjectButton) {
            DatasetObjectPtr obj = scapeDatasetPtr->get_object_by_name(objectNameComboBox->currentText().toStdString());


            for (auto &dp:obj->data_points)
                if (dp.gts.size() > 1)
                    tot_n_dp++;
            progressBar->setMaximum(tot_n_dp);

            group_vis->clear();
            for (auto &dp:obj->data_points) {
                if (dp.gts.size() > 1) {
                    run_enabled_algorithms(geneticEvaluatorOCPtr, obj, dp, rawDataMapAlgObjVec);
                    progressBar->setValue(++n_dp);
                    QCoreApplication::processEvents();
                }
            }

        } else if (button == runEnabledOnDatasetButton) {
            group_vis->clear();

            for (auto &obj:scapeDatasetPtr->objects)
                for (auto &dp:obj->data_points)
                    if (dp.gts.size() > 1)
                        tot_n_dp++;

            for (auto &obj:scapeDatasetPtr->objects) {
                for (auto &dp:obj->data_points) {
                    if (dp.gts.size() > 1) {
                        run_enabled_algorithms(geneticEvaluatorOCPtr, obj, dp, rawDataMapAlgObjVec);
                        progressBar->setValue(++n_dp);
                        QCoreApplication::processEvents();
                    }
                }
            }
        }
        progressBar->hide();

        algorithmDataProc = AlgorithmDataProc(rawDataMapAlgObjVec, general_settings.ground_truth_t_thresh->value(),
                                              general_settings.ground_truth_r_thresh->value());
        dataProcSaveResultsButton->setEnabled(true);


        // Clear and update visualizer
        group_vis->clear();
        group_vis->addIdPointCloud(geneticEvaluatorOCPtr->pc, "Captured Point Cloud");
        // Add gt and data to visualization
        auto obj = scapeDatasetPtr->get_object_by_name(algorithmDataProc.objNameVec.back());
        auto &dp = rawDataMapAlgObjVec.begin()->second[obj].back().dp;
        PointCloudT::Ptr meshpc = obj->get_mesh_point_cloud();
        for (int g = 0; g < dp.gts.size(); g++) {
            PointCloudT::Ptr gtpc_vis = pcl::make_shared<PointCloudT>();
            std::string id = "gt_" + std::to_string(g);
            pcl::transformPointCloud(*meshpc, *gtpc_vis, dp.gts[g]);
            group_vis->addIdPointCloud(gtpc_vis, id, "Ground Truth", 0, 255, 255);
        }

        for (auto &alg:algorithms) {

            size_t bi, ei;
            algorithmDataProc.get_begin_end_index(bi, ei, alg->name, obj->name);

            std::vector<int> tp, tn, fp, fn;
            algorithmDataProc.getFPTN(tp, tn, fp, fn, *(algorithmDataProc.chromosomeVec.begin() + ei - 1),
                                      *(algorithmDataProc.trueOCVec.begin() + ei - 1));
            add_results_to_visualizer(geneticEvaluatorOCPtr, *(algorithmDataProc.algNameVec.begin() + ei - 1),
                                      *(algorithmDataProc.algNameVec.begin() + ei - 1), tp, tn, fp, fn);
        }
        group_vis->resetCamera();
        group_vis->update_text();

        // Generate Result Table
        std::vector<std::string> dataTypes = {"Acc", "Rec", "Pre"};
        resultTableWidget->clear();
        resultTableWidget->setColumnCount(algorithmDataProc.algNameUniqueVec.size());
        resultTableWidget->setRowCount(dataTypes.size());

        QStringList horizontalHeaders, verticalHeaders;
        std::transform(algorithmDataProc.algNameUniqueVec.begin(), algorithmDataProc.algNameUniqueVec.end(),
                       std::back_inserter(horizontalHeaders), [](std::string &s) { return QString::fromStdString(s); });
        std::transform(dataTypes.begin(), dataTypes.end(), std::back_inserter(verticalHeaders),
                       [](std::string &s) { return QString::fromStdString(s); });
        resultTableWidget->setHorizontalHeaderLabels(horizontalHeaders);
        resultTableWidget->setVerticalHeaderLabels(verticalHeaders);

        int col_n = 0;
        for (auto &alg:algorithmDataProc.algNameUniqueVec) {
            resultTableWidget->setCellWidget(0, col_n, new QLabel(QString::fromStdString(
                    std::to_string(algorithmDataProc.get_avr(algorithmDataProc.accuracyVec, alg)))));
            resultTableWidget->setCellWidget(1, col_n, new QLabel(QString::fromStdString(
                    std::to_string(algorithmDataProc.get_avr(algorithmDataProc.recallVec, alg)))));
            resultTableWidget->setCellWidget(2, col_n, new QLabel(QString::fromStdString(
                    std::to_string(algorithmDataProc.get_avr(algorithmDataProc.precisionVec, alg)))));
            col_n++;
        }

//        for(auto &alg:algorithmDataProc.algNameUniqueVec){
//            double avr_alg_acc =algorithmDataProc.get_avr(algorithmDataProc.accuracyVec, alg);
//            dataProcFormLayout->addRow(new QLabel(QString::fromStdString(alg)),new QLabel(QString::fromStdString(std::to_string(avr_alg_acc))));
//            std::cout<<"avr_alg_acc: "<<avr_alg_acc<<std::endl;
//            for(auto &obj:algorithmDataProc.objNameUniqueVec){
//                double avr_obj_acc =algorithmDataProc.get_avr(algorithmDataProc.accuracyVec, alg,obj);
//                dataProcFormLayout->addRow(new QLabel(QString::fromStdString(obj)),new QLabel(QString::fromStdString(std::to_string(avr_obj_acc))));
//                std::cout<<"avr_obj_acc: "<<avr_obj_acc<<std::endl;
//            }
//        }

        dataProcDock->show();
        resultDock->show();
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

void AlgorithmTuner::loadSettings() {
    QSettings qsettings;
    settings.data_folder = qsettings.value("data_folder", "/home").toString();
    settings.recognition_folder = qsettings.value("recognition_folder", "/home").toString();
    settings.data_save_file = qsettings.value("data_save_folder", "/home").toString();

    general_settings.ground_truth_t_thresh->setValue(
            qsettings.value("general_settings.ground_truth_t_thresh", "4").toDouble());
    general_settings.ground_truth_r_thresh->setValue(
            qsettings.value("general_settings.ground_truth_r_thresh", "5").toDouble());

    for (auto &alg:algorithms)
        alg->load_settings(qsettings);

    loadEvaluatorSettings();
}

void AlgorithmTuner::saveSettings() {
    QSettings qsettings;
    qsettings.setValue("data_folder", settings.data_folder);
    qsettings.setValue("recognition_folder", settings.recognition_folder);
    qsettings.setValue("data_save_folder", settings.data_save_file);

    qsettings.setValue("general_settings.ground_truth_t_thresh", general_settings.ground_truth_t_thresh->value());
    qsettings.setValue("general_settings.ground_truth_r_thresh", general_settings.ground_truth_r_thresh->value());

    for (auto &alg:algorithms)
        alg->save_settings(qsettings);

    saveEvaluatorSettings();
}

AlgorithmTuner::~AlgorithmTuner() {
    saveSettings();
    delete this;
}

void AlgorithmTuner::save_data() {
    auto last_save_folder = std::filesystem::path(settings.data_save_file.toStdString()).parent_path();
    QString proposed_file_name = QString::fromStdString(
            (last_save_folder / (getTimeString("%Y-%d-%m-%H-%M-%S") + "-AlgorithmTunerData.csv")).string());
    settings.data_save_file = QFileDialog::getSaveFileName(this, "Save Data", proposed_file_name);

    algorithmDataProc.save_data(settings.data_save_file.toStdString());
}

void AlgorithmTuner::bar_plot() {
    std::string plotDataType = plotComboBox->currentText().toStdString();
    if (plotDataType == "Accuracy")
        algorithmDataProc.bar_plot(algorithmDataProc.accuracyVec);
    if (plotDataType == "Precision")
        algorithmDataProc.bar_plot(algorithmDataProc.precisionVec);
    if (plotDataType == "Recall")
        algorithmDataProc.bar_plot(algorithmDataProc.recallVec);
    if (plotDataType == "TP")
        algorithmDataProc.bar_plot(algorithmDataProc.tpVec);
    if (plotDataType == "TN")
        algorithmDataProc.bar_plot(algorithmDataProc.tnVec);
    if (plotDataType == "FP")
        algorithmDataProc.bar_plot(algorithmDataProc.fpVec);
    if (plotDataType == "FN")
        algorithmDataProc.bar_plot(algorithmDataProc.fnVec);
}


