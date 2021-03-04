#include "algorithm_tuner.hpp"
#include "hypothesis_verification/hv_alg/ga.hpp"
#include "../../lib/HypothesisVerificaiton/include/hypothesis_verification/hv_alg/sequential_prior.hpp"
#include "dataset/transform_utility.hpp"
#include <dataset/scape/ScapeDataset.hpp>
#include <hypothesis_verification/evaluator/point_cloud_renderer.hpp>


//Evaluator


//STL
#include <iostream>
#include <sstream>

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
    QObject::connect(algComboBox, qOverload<const QString &>(&QComboBox::currentIndexChanged), this,
                     qOverload<const QString &>(&AlgorithmTuner::update_range_params));
    QObject::connect(paramComboBox, qOverload<const QString &>(&QComboBox::currentIndexChanged), this,
                     qOverload<const QString &>(&AlgorithmTuner::update_range_param_limits));

//    algComboBox
    QObject::connect(datapointSpinBox, qOverload<int>(&QSpinBox::valueChanged), this,
                     qOverload<int>(&AlgorithmTuner::update_ocs_and_gts));
    QObject::connect(runEnabledMethodsButton, &QPushButton::pressed, this,
                     qOverload<>(&AlgorithmTuner::run_enabled_algorithms));
    QObject::connect(dataProcSaveResultsButton, &QPushButton::pressed, this, &AlgorithmTuner::save_data);
    QObject::connect(tunedParamSavePushButton, &QPushButton::pressed, this, &AlgorithmTuner::save_tuned_parameters);
    QObject::connect(tunedParamLoadPushButton, &QPushButton::pressed, this, &AlgorithmTuner::load_tuned_parameters);
    QObject::connect(evaluatorTabWidget, &QTabWidget::currentChanged, this, &AlgorithmTuner::update_evaluator);

    // Disable button that arent currently usable
    runEnabledMethodsButton->setEnabled(false);
    dataProcSaveResultsButton->setEnabled(false);

    // Hide docks and widgets
    datasetDock->hide();
    resultDock->hide();
    dataProcDock->hide();
    progressBar->hide();

    // Create algorithms and evaluator interfaces and add variables to GUI
    hv_algorithms.push_back(std::make_shared<GAInterface>());
    hv_algorithms.push_back(std::make_shared<GAWInterface>());
    hv_algorithms.push_back(std::make_shared<GASPInterface>());
    hv_algorithms.push_back(std::make_shared<SPInterface>());
    hv_algorithms.push_back(std::make_shared<BaselineInterface>());
    hv_algorithms.push_back(std::make_shared<RandomInterface>());
    algorithms.insert(algorithms.end(), hv_algorithms.begin(), hv_algorithms.end());
    evaluators.push_back(std::make_shared<GeneticEvaluatorInlierCollisionInterface>());
    evaluators.push_back(std::make_shared<GeneticEvaluatorInlierCollisionScaledInterface>());
    evaluators.push_back(std::make_shared<GeneticEvaluatorScoreCollisionInterface>());
    evaluators.push_back(std::make_shared<GeneticEvaluatorUniqueInlierCollisionScaledInterface>());
    evaluators.push_back(std::make_shared<GeneticEvaluatorF1Interface>());
    evaluators.push_back(std::make_shared<GeneticEvaluatorPrecisionInterface>());
    algorithms.insert(algorithms.end(), evaluators.begin(), evaluators.end());

    data_info_doc = std::make_shared<rapidcsv::CSVDoc>("", rapidcsv::LabelParams(0, -1));
    data_doc = std::make_shared<rapidcsv::CSVDoc>("", rapidcsv::LabelParams(0, -1));

    for (auto &alg:algorithms)
        algComboBox->addItem(QString::fromStdString(alg->name));

    loadSettings(); // Load setting from prev session e.g. paths for dataset

    for (auto &alg:hv_algorithms)
        alg->add_parameters_to_tabwidget(hvAlgTabWidget);
    for (auto &alg:evaluators)
        alg->add_parameters_to_tabwidget(evaluatorTabWidget);


    //Add general Settings to GUI
    generalSettingsFormLayout->addRow(QString::fromStdString("ground_truth_t_thresh"),
                                      general_settings.ground_truth_t_thresh);
    generalSettingsFormLayout->addRow(QString::fromStdString("ground_truth_r_thresh"),
                                      general_settings.ground_truth_r_thresh);


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


void AlgorithmTuner::run_enabled_algorithms(GeneticEvaluatorPtr &geneticEvaluatorPtr, DatasetObjectPtr &obj, int &dpI) {
    DataPoint &dp = obj->data_points[dpI];
    if (geneticEvaluatorPtr->datasetObjectPtr == nullptr) {
        geneticEvaluatorPtr->initialise_object(obj, dp);
    } else if (geneticEvaluatorPtr->datasetObjectPtr->name != obj->name) {
        geneticEvaluatorPtr->initialise_object(obj, dp);
    } else if ((geneticEvaluatorPtr->dp.oc_scores != dp.oc_scores) ||
               (geneticEvaluatorPtr->dp.ground_truth_path != dp.ground_truth_path)) {
        geneticEvaluatorPtr->initialise_datapoint(dp);
    }

    HVResult hvResult;
    int row_i = data_doc->GetRowCount();
    for (auto &alg:hv_algorithms) {
        alg->update_parameters();
        current_evaluator->update_parameters();
        if (alg->enabled) {
            for(int i = 0;i<repetitionSpinBox->value();i++)
            {
                alg->run(geneticEvaluatorPtr, hvResult);
                algorithmDataProc.append_processed_data_to_doc(data_doc, row_i, alg->name, obj, dpI, hvResult);
                set_doc_alg_variables(data_doc, row_i++);
            }
        }
    }
}

std::vector<double> AlgorithmTuner::get_param_test_values() {
    std::vector<double> param_test_values;

    double paramTestMin = paramMinDoubleSpinBox->value();
    double paramTestMax = paramMaxDoubleSpinBox->value();
    double paramTestStep = paramStepDoubleSpinBox->value();

    param_test_values.resize(static_cast<int>((paramTestMax - paramTestMin) / paramTestStep) + 1);
    std::iota(param_test_values.begin(), param_test_values.end(), 0); // Generate step indices
    std::transform(param_test_values.begin(), param_test_values.end(), param_test_values.begin(),
                   [paramTestMin, paramTestStep](double &c) {
                       return paramTestMin + c * paramTestStep;
                   }); // multiply each step with step index length
    return param_test_values;
}


void AlgorithmTuner::run_enabled_algorithms() {
    bool algorithms_enabled = false;
    int n_enabled_algs = 0;
    for (auto &alg:hv_algorithms) {
        alg->update_parameters();
        if (alg->enabled) {
            n_enabled_algs++;
            algorithms_enabled = true;
        }
    }

    if (algorithms_enabled && (current_evaluator!= nullptr)) {
        data_doc->clear();

        progressBar->show();
        algorithmDataProc = AlgorithmDataProc(general_settings.ground_truth_t_thresh->value(),
                                              general_settings.ground_truth_r_thresh->value());
        algorithmDataProc.set_column_names(data_doc);
        set_doc_alg_var_names(data_doc);

        if (scapeDatasetPtr != nullptr) {
            std::vector<double> param_test_values;
            if (paramComboBox->count() > 0)
                param_test_values = get_param_test_values();
            double initial_param_test_value = getSpinBoxWidgetValue(parameter_test_widget);

            std::vector<std::pair<DatasetObjectPtr, std::vector<int>>> object_and_datapoint_index_pairs;
            if (runEnabledMethodsComboBox->currentIndex() == 0) {
                DatasetObjectPtr obj = scapeDatasetPtr->get_object_by_name(
                        objectNameComboBox->currentText().toStdString());
                object_and_datapoint_index_pairs.emplace_back(obj, std::vector{datapointSpinBox->value()});
            } else if (runEnabledMethodsComboBox->currentIndex() == 1) {
                DatasetObjectPtr obj = scapeDatasetPtr->get_object_by_name(
                        objectNameComboBox->currentText().toStdString());
                object_and_datapoint_index_pairs.emplace_back(obj, std::vector<int>{});
                for (int i = 0; i < obj->data_points.size(); i++) {
                    if (obj->data_points[i].gts.size() > 1) {
                        object_and_datapoint_index_pairs.back().second.emplace_back(i);
                    }
                }
            } else if (runEnabledMethodsComboBox->currentIndex() == 2) {
                for (auto &obj:scapeDatasetPtr->objects) {
                    object_and_datapoint_index_pairs.emplace_back(obj, std::vector<int>{});
                    for (int i = 0; i < obj->data_points.size(); i++) {
                        if (obj->data_points[i].gts.size() > 1) {
                            object_and_datapoint_index_pairs.back().second.emplace_back(i);
                        }
                    }
                }
            }


            EvaluatorInterfacePtr evaluatorInterfacePtr = current_evaluator;
            evaluatorInterfacePtr->update_parameters();
            GeneticEvaluatorPtr &geneticEvaluatorPtr = evaluatorInterfacePtr->geneticEvaluatorPtr;


            int n_dp = 0, tot_n_dp = 0;
            for (auto &obj_dp_pair:object_and_datapoint_index_pairs) {
                tot_n_dp += obj_dp_pair.second.size();
            }

            // Attempt to speed up saving data
            if(param_test_values.size())
                data_doc->mData.reserve(tot_n_dp*n_enabled_algs*param_test_values.size());
            else
                data_doc->mData.reserve(tot_n_dp*n_enabled_algs);
            for(auto&v:data_doc->mData)
                v.reserve(data_doc->GetColumnCount());


            progressBar->setMaximum(tot_n_dp);
            progressBar->show();


            read_tuned_parameters();
            if (paramComboBox->count() > 0) {
                bool first_run = true;
                for (auto &obj_dpi_pair:object_and_datapoint_index_pairs) {
                    update_tuned_parameters(obj_dpi_pair.first);
                    for (auto &dpI:obj_dpi_pair.second) {
                        for (int i = 0; i < param_test_values.size(); i++) {
                            auto &val = param_test_values[i];
                            setSpinBoxWidgetValue(parameter_test_widget, val);
                            evaluatorInterfacePtr->update_parameters();
                            run_enabled_algorithms(geneticEvaluatorPtr, obj_dpi_pair.first, dpI);
                            progressBar->setValue(++n_dp / param_test_values.size());
                            QCoreApplication::processEvents();
                        }
                    }
                }
                setSpinBoxWidgetValue(parameter_test_widget, initial_param_test_value);
            } else {
                for (auto &obj_dpi_pair:object_and_datapoint_index_pairs) {
                    update_tuned_parameters(obj_dpi_pair.first);
                    for (auto &dpI:obj_dpi_pair.second) {
                        evaluatorInterfacePtr->update_parameters();
                        run_enabled_algorithms(geneticEvaluatorPtr, obj_dpi_pair.first, dpI);
                        progressBar->setValue(++n_dp);
                        QCoreApplication::processEvents();
                    }
                }
            }


            progressBar->hide();
            dataProcSaveResultsButton->setEnabled(true);

            // Clear and update visualizer
            group_vis->clear();
            group_vis->addIdPointCloud(geneticEvaluatorPtr->pc, "Captured Point Cloud");
            // Add gt and data to visualization
            std::string objName = data_doc->GetCell<std::string>(algorithmDataProc.column_name_indices["objName"],
                                                                 data_doc->GetRowCount() - 1);
            int dpI = data_doc->GetCell<int>(algorithmDataProc.column_name_indices["dpI"], data_doc->GetRowCount() - 1);


            auto obj = scapeDatasetPtr->get_object_by_name(objName);
            auto &dp = obj->data_points[dpI];
            PointCloudT::Ptr meshpc = obj->get_mesh_point_cloud();

            auto var_itt = std::find_if(evaluatorInterfacePtr->parameters_d.begin(),
                                        evaluatorInterfacePtr->parameters_d.end(),
                                        [](param_d &var) { return var.name == "VoxelGrid leaf size"; });//
            if (var_itt != evaluatorInterfacePtr->parameters_d.end()) {
                GeneticEvaluatorInlierCollisionPtr geneticEvaluatorOcPtr = std::dynamic_pointer_cast<GeneticEvaluatorInlierCollision>(
                        geneticEvaluatorPtr);
                for (int g = 0; g < dp.gts.size(); g++) {
                    PointCloudT::Ptr gtpc_vis = pcl::make_shared<PointCloudT>();
                    std::string id = "gt_" + std::to_string(g);
                    pcl::transformPointCloud(*meshpc, *gtpc_vis, dp.gts[g]);
                    geneticEvaluatorOcPtr->voxelGridPtr->setInputCloud(gtpc_vis);
                    geneticEvaluatorOcPtr->voxelGridPtr->setLeafSize(*(var_itt->val), *(var_itt->val), *(var_itt->val));
                    geneticEvaluatorOcPtr->voxelGridPtr->filter(*gtpc_vis);
                    group_vis->addIdPointCloud(gtpc_vis, id, "Ground Truth", 0, 255, 255);
                }
            } else {
                for (int g = 0; g < dp.gts.size(); g++) {
                    PointCloudT::Ptr gtpc_vis = pcl::make_shared<PointCloudT>();
                    std::string id = "gt_" + std::to_string(g);
                    pcl::transformPointCloud(*meshpc, *gtpc_vis, dp.gts[g]);
                    group_vis->addIdPointCloud(gtpc_vis, id, "Ground Truth", 0, 255, 255);
                }
            }
            group_vis->toggle_group_opacity(group_vis->find_pcv_group_id("Ground Truth"));

            for (auto &alg:hv_algorithms) {
                if (alg->enabled) {
                    std::vector<int> correct_oc_i;
                    std::vector<double> t_dists, r_dists;
                    tu::find_correct_ocs(dp.ocs, dp.gts, algorithmDataProc.t_thresh, algorithmDataProc.r_thresh,correct_oc_i, t_dists, r_dists, obj->symmetry_transforms);
                    chromosomeT chromosome;
                    for(int i = data_doc->GetRowCount()-1;i>=0;i--) {
                        if(alg->name==data_doc->GetCell<std::string>(algorithmDataProc.column_name_indices["algName"],  i)) {
                            chromosome = data_doc->GetCell<chromosomeT>(algorithmDataProc.column_name_indices["chromosome"], i);
                            break;
                        }
                    }
                    std::vector<int> tp, tn, fp, fn;
                    tu::getFPTN(tp, tn, fp, fn, chromosome, correct_oc_i);
                    add_results_to_visualizer(geneticEvaluatorPtr, alg->name, alg->name, tp, tn, fp, fn);
                }
            }
            group_vis->resetCamera();
            group_vis->update_text();

            dataProcDock->show();
            resultDock->show();
        }
    }
}


void AlgorithmTuner::add_results_to_visualizer(GeneticEvaluatorPtr &geneticEvaluatorPtr, std::string group,
                                               std::string node_prefix, std::vector<int> tp, std::vector<int> tn,
                                               std::vector<int> fp, std::vector<int> fn) {

    int n_ocs = geneticEvaluatorPtr->dp.ocs.size();
    PointCloudRenderer pc_render;
    pc_render.addActorsPLY(geneticEvaluatorPtr->datasetObjectPtr->mesh_path, geneticEvaluatorPtr->dp.ocs);
    pc_render.fitCameraAndResolution();
    std::vector<PointCloudT::Ptr> visible_oc_pcs;
    visible_oc_pcs.clear();
    visible_oc_pcs.reserve(n_ocs);
    pc_render.renderPointClouds(visible_oc_pcs);


    for (int i = 0; i < n_ocs; i++) {
        std::string id = node_prefix + "_oc_" + std::to_string(i);
        PointCloudT::Ptr pcPtr= nullptr;

        if (std::find(tp.begin(), tp.end(), i) != tp.end()) {
            group_vis->addIdPointCloud(visible_oc_pcs[i], id,
                                       group + "/True Positives", 0, 255, 0);
        } else if (std::find(fp.begin(), fp.end(), i) != fp.end()) {
            group_vis->addIdPointCloud(visible_oc_pcs[i], id,
                                       group + "/False Positives", 255, 0, 0);
        } else if (std::find(tn.begin(), tn.end(), i) != tn.end()) {
            group_vis->addIdPointCloud(visible_oc_pcs[i], id,
                                       group + "/True Negatives", 255, 255, 0);
        } else if (std::find(fn.begin(), fn.end(), i) != fn.end()) {
            group_vis->addIdPointCloud(visible_oc_pcs[i], id,
                                       group + "/False Negatives", 0, 0, 255);
        }
    }
}


void AlgorithmTuner::timeoutSlot() {
    vtkWidget->update();
}

EvaluatorInterfacePtr AlgorithmTuner::get_evaluator_interface(QString name) {
    return get_evaluator_interface(name.toStdString());
}

EvaluatorInterfacePtr AlgorithmTuner::get_evaluator_interface(std::string name) {
    auto eval_itt = std::find_if(evaluators.begin(), evaluators.end(),
                                 [name](EvaluatorInterfacePtr &evaluatorInterfacePtr) {
                                     return evaluatorInterfacePtr->name == name;
                                 });
    if (eval_itt != evaluators.end())
        return *eval_itt;
    else
        return nullptr;
}

void AlgorithmTuner::loadSettings() {
    QSettings qsettings;
    settings.data_folder = qsettings.value("data_folder", "/home").toString();
    settings.recognition_folder = qsettings.value("recognition_folder", "/home").toString();
    settings.data_save_file = qsettings.value("data_save_folder", "/home").toString();
    settings.tuned_params_file = qsettings.value("tuned_params_file", "/home").toString();

    general_settings.ground_truth_t_thresh->setValue(
            qsettings.value("general_settings.ground_truth_t_thresh", "4").toDouble());
    general_settings.ground_truth_r_thresh->setValue(
            qsettings.value("general_settings.ground_truth_r_thresh", "5").toDouble());

    for (auto &alg:hv_algorithms)
        alg->load_settings(qsettings);
    for (auto &alg:evaluators)
        alg->load_settings(qsettings);
}

void AlgorithmTuner::saveSettings() {
    QSettings qsettings;
    qsettings.setValue("data_folder", settings.data_folder);
    qsettings.setValue("recognition_folder", settings.recognition_folder);
    qsettings.setValue("data_save_folder", settings.data_save_file);
    qsettings.setValue("tuned_params_file", settings.tuned_params_file);

    qsettings.setValue("general_settings.ground_truth_t_thresh", general_settings.ground_truth_t_thresh->value());
    qsettings.setValue("general_settings.ground_truth_r_thresh", general_settings.ground_truth_r_thresh->value());

    for (auto &alg:hv_algorithms)
        alg->save_settings(qsettings);
    for (auto &alg:evaluators)
        alg->save_settings(qsettings);
}

AlgorithmTuner::~AlgorithmTuner() {
    saveSettings();
    delete this;
}

void AlgorithmTuner::save_data() {
    auto last_save_folder = std::filesystem::path(settings.data_save_file.toStdString()).parent_path();

    std::string time_string = getTimeString("%Y-%d-%m-%H-%M-%S");
    QString proposed_file_name = QString::fromStdString(
            (last_save_folder / (time_string + "-AlgorithmTunerData.csv")).string());
    settings.data_save_file = QFileDialog::getSaveFileName(this, "Save Dynamic Data", proposed_file_name);

    auto path = std::filesystem::path(settings.data_save_file.toStdString());
    auto filename = std::filesystem::path(settings.data_save_file.toStdString()).filename().replace_extension();
    auto parent_folder = std::filesystem::path(settings.data_save_file.toStdString()).parent_path();


    data_doc->Save(path.replace_filename(time_string + "-data.csv"));
    data_info_doc->SetCell(0, 0, data_doc->mPath);
    data_info_doc->Save(path.replace_filename(time_string + "-data_info.csv"));
}


void AlgorithmTuner::update_range_params(const QString &s) {
    paramComboBox->clear();

    auto it = std::find_if(algorithms.begin(), algorithms.end(),
                           [&s](const AlgorithmInterfacePtr &algIPtr) { return (*algIPtr) == s.toStdString(); });

    if (it != algorithms.end()) {
        for (auto &param:(*it)->parameters_i)
            paramComboBox->addItem(QString::fromStdString(param.name));
        for (auto &param:(*it)->parameters_d)
            paramComboBox->addItem(QString::fromStdString(param.name));
    }
}

void AlgorithmTuner::update_range_param_limits(const QString &s) {

    auto it = std::find_if(algorithms.begin(), algorithms.end(), [this](const AlgorithmInterfacePtr &algIPtr) {
        return (*algIPtr) == algComboBox->currentText().toStdString();
    });
    if (it != algorithms.end()) {
        auto it_i = std::find_if((*it)->parameters_i.begin(), (*it)->parameters_i.end(),
                                 [s](const param_i &vi) { return vi.name == s.toStdString(); });
        auto it_d = std::find_if((*it)->parameters_d.begin(), (*it)->parameters_d.end(),
                                 [s](const param_d &vd) { return vd.name == s.toStdString(); });

        if (it_i != (*it)->parameters_i.end()) {
            paramMinDoubleSpinBox->setDecimals(0);
            paramMaxDoubleSpinBox->setDecimals(0);
            paramStepDoubleSpinBox->setDecimals(0);
            parameter_test_widget = it_i->spinBox;
            parameter_test_name = it_i->name;
        }
        if (it_d != (*it)->parameters_d.end()) {
            paramMinDoubleSpinBox->setDecimals(3);
            paramMaxDoubleSpinBox->setDecimals(3);
            paramStepDoubleSpinBox->setDecimals(3);
            parameter_test_widget = it_d->spinBox;
            parameter_test_name = it_d->name;
        }
    }
}


void AlgorithmTuner::setSpinBoxWidgetValue(QWidget *widget, double val) {
    if (QSpinBox *sb = qobject_cast<QSpinBox *>(widget))
        sb->setValue(val);
    if (QDoubleSpinBox *sb = qobject_cast<QDoubleSpinBox *>(widget))
        sb->setValue(val);
}

double AlgorithmTuner::getSpinBoxWidgetValue(QWidget *widget) {
    if (QSpinBox *sb = qobject_cast<QSpinBox *>(widget))
        return sb->value();
    if (QDoubleSpinBox *sb = qobject_cast<QDoubleSpinBox *>(widget))
        return sb->value();

    return -1;
}

void AlgorithmTuner::set_doc_alg_var_names(rapidcsv::CSVDocPtr &csvDoc) {
    enabled_alg_variables_csv_start_col = csvDoc->GetColumnCount();
    n_enabled_alg_variables = enabled_alg_variables_csv_start_col;

    csvDoc->SetColumnName(n_enabled_alg_variables++, "geName");
    csvDoc->SetColumnName(n_enabled_alg_variables++, "t_thresh");
    csvDoc->SetColumnName(n_enabled_alg_variables++, "r_thresh");

    if (current_evaluator != nullptr) {
        current_evaluator->update_parameters();
        for (auto vi:current_evaluator->parameters_i)
            csvDoc->SetColumnName(n_enabled_alg_variables++, current_evaluator->name + "/" + vi.name);
        for (auto vd:current_evaluator->parameters_d)
            csvDoc->SetColumnName(n_enabled_alg_variables++, current_evaluator->name + "/" + vd.name);
        for (auto vb:current_evaluator->parameters_b)
            csvDoc->SetColumnName(n_enabled_alg_variables++, current_evaluator->name + "/" + vb.name);
    }
    for (auto alg:hv_algorithms) {
        alg->update_parameters();
        if (alg->enabled) {
            for (auto vi:alg->parameters_i)
                csvDoc->SetColumnName(n_enabled_alg_variables++, alg->name + "/" + vi.name);
            for (auto vd:alg->parameters_d)
                csvDoc->SetColumnName(n_enabled_alg_variables++, alg->name + "/" + vd.name);
            for (auto vb:alg->parameters_b)
                csvDoc->SetColumnName(n_enabled_alg_variables++, alg->name + "/" + vb.name);
        }
    }

    n_enabled_alg_variables -= enabled_alg_variables_csv_start_col;

}

void AlgorithmTuner::set_doc_alg_variables(rapidcsv::CSVDocPtr &csvDoc, int row_i) {

    int col_i = enabled_alg_variables_csv_start_col;

    csvDoc->SetCell(col_i++, row_i, evaluatorTabWidget->tabText(evaluatorTabWidget->currentIndex()).toStdString());
    csvDoc->SetCell(col_i++, row_i, general_settings.ground_truth_t_thresh->value());
    csvDoc->SetCell(col_i++, row_i, general_settings.ground_truth_r_thresh->value());

    if (current_evaluator != nullptr) {
        current_evaluator->update_parameters();
        for (auto vi:current_evaluator->parameters_i)
            csvDoc->SetCell(col_i++, row_i, *vi.val);
        for (auto vd:current_evaluator->parameters_d)
            csvDoc->SetCell(col_i++, row_i, *vd.val);
        for (auto vb:current_evaluator->parameters_b)
            csvDoc->SetCell(col_i++, row_i, *vb.val);
    }
    for (auto alg:hv_algorithms) {
        alg->update_parameters();
        if (alg->enabled) {
            for (auto vi:alg->parameters_i)
                csvDoc->SetCell(col_i++, row_i, *vi.val);
            for (auto vd:alg->parameters_d)
                csvDoc->SetCell(col_i++, row_i, *vd.val);
            for (auto vb:alg->parameters_b)
                csvDoc->SetCell(col_i++, row_i, *vb.val);
        }
    }
    if (col_i > enabled_alg_variables_csv_start_col + n_enabled_alg_variables)
        std::cout << "More variables was saved to csv than was registered in set_doc_alg_var_names" << std::endl;
}

void AlgorithmTuner::load_tuned_parameters() {
    settings.tuned_params_file = QFileDialog::getOpenFileName(this,"Chosen file with tuned parameters",settings.tuned_params_file);
    QFile file(settings.tuned_params_file);

    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        tunedParamstextEdit->setText(file.readAll());
    }
}

void AlgorithmTuner::save_tuned_parameters() {
    ofstream newfile("./tmpfile");
    newfile << tunedParamstextEdit->toPlainText().toStdString();
    newfile.close();
    std::remove(settings.tuned_params_file.toStdString().c_str());
    std::rename("./tmpfile", settings.tuned_params_file.toStdString().c_str());
}

void AlgorithmTuner::update_tuned_parameters(DatasetObjectPtr &objPtr) {
    for(auto &widget_val_pair : object_tuned_variables[objPtr])
        setSpinBoxWidgetValue(widget_val_pair.first,widget_val_pair.second);
}

void AlgorithmTuner::read_tuned_parameters() {
    if(tunedParamcheckBox->isChecked()) {
        object_tuned_variables.clear();
        std::string tuned_parameter_s = tunedParamstextEdit->toPlainText().toStdString();
        std::stringstream tuned_parameter_ss(tuned_parameter_s);
        std::string line, word;
        for (; std::getline(tuned_parameter_ss, line);) {
            std::stringstream line_ss(line);
            std::array<std::string, 4> read_words;
            for (int i = 0; i < 4; i++) {
                std::getline(line_ss, read_words[i], ',');
            }
            if (!read_words[3].empty()) {
                auto &obj_s = read_words[0];
                auto &alg_s = read_words[1];
                auto &param_s = read_words[2];
                auto &val_s = read_words[3];

                auto obj_it = std::find_if(scapeDatasetPtr->objects.begin(), scapeDatasetPtr->objects.end(),
                                           [obj_s](DatasetObjectPtr &objPtr) { return objPtr->name == obj_s; });
                if (obj_it != scapeDatasetPtr->objects.end()) {
                    auto alg_it = std::find_if(algorithms.begin(), algorithms.end(),
                                               [alg_s](AlgorithmInterfacePtr &algPtr) {
                                                   return algPtr->name == alg_s;
                                               });
                    if (alg_it != algorithms.end()) {
                        auto &algPtr = *alg_it;
                        for (auto &param_b: algPtr->parameters_b) {
                            if (param_b.name == param_s) {
                                object_tuned_variables[*obj_it].emplace_back(param_b.checkBox, std::stod(val_s));
                            }
                        }
                        for (auto &param_i: algPtr->parameters_i) {
                            if (param_i.name == param_s) {
                                object_tuned_variables[*obj_it].emplace_back(param_i.spinBox, std::stod(val_s));
                            }
                        }
                        for (auto &param_d: algPtr->parameters_d) {
                            if (param_d.name == param_s) {
                                object_tuned_variables[*obj_it].emplace_back(param_d.spinBox, std::stod(val_s));
                            }
                        }
                    }
                }
            }
        }
    }
}

void AlgorithmTuner::update_evaluator(const int &i) {
    current_evaluator = get_evaluator_interface(evaluatorTabWidget->tabText(i));
}

