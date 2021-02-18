#include "algorithm_tuner.hpp"
#include "hypothesis_verification/hv_alg/ga.hpp"
#include "../../lib/HypothesisVerificaiton/include/hypothesis_verification/hv_alg/sequential_prior.hpp"
#include "dataset/transform_utility.hpp"
#include <dataset/scape/ScapeDataset.hpp>


//Evaluator


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
    QObject::connect(barPlotButton, &QPushButton::pressed, this, &AlgorithmTuner::bar_plot);
    QObject::connect(paramTestPlotButton, &QPushButton::pressed, this, &AlgorithmTuner::param_test_plot);

    QObject::connect(evaluator_types_combo_box, &QComboBox::currentTextChanged, this,
                     &AlgorithmTuner::update_evaluator_type);

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
    algorithms.insert(algorithms.end(),hv_algorithms.begin(),hv_algorithms.end());
    evaluators.push_back(std::make_shared<GeneticEvaluatorOCInterface>());
    algorithms.insert(algorithms.end(),evaluators.begin(),evaluators.end());

    data_info_doc = std::make_shared<rapidcsv::CSVDoc>("",rapidcsv::LabelParams(0, -1));

    for(auto &alg:algorithms)
        algComboBox->addItem(QString::fromStdString(alg->name));

    loadSettings(); // Load setting from prev session e.g. paths for dataset

    for(auto &alg:hv_algorithms)
        alg->add_variable_to_tabwidget(settingsTabWidget);

    //Add Evaluator Settings to GUI
    QString temp_current_evaluator_text=current_evaluator_text;
    for(auto &alg:evaluators) {
        evaluator_types_combo_box->addItem(QString::fromStdString(alg->name));
        if(alg->name==temp_current_evaluator_text.toStdString())
            evaluator_types_combo_box->setCurrentIndex(evaluator_types_combo_box->count()-1);
    }

    for(int i = 0;i<evaluator_types_combo_box->count();i++){
        if(evaluator_types_combo_box->itemText(i)==current_evaluator_text){
            evaluator_types_combo_box->setCurrentIndex(i);
        }
    }

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


void AlgorithmTuner::run_enabled_algorithms(GeneticEvaluatorPtr &geneticEvaluatorPtr, DatasetObjectPtr &obj,
                                            DataPoint &dp, rawDataMapAlgObjVecT &rawDataMapAlgObjVec) {
    if (geneticEvaluatorPtr->datasetObjectPtr == nullptr) {
        geneticEvaluatorPtr->initialise_object(obj, dp);
    } else if (geneticEvaluatorPtr->datasetObjectPtr->name != obj->name) {
        geneticEvaluatorPtr->initialise_object(obj, dp);
    } else if ((geneticEvaluatorPtr->dp.oc_scores != dp.oc_scores) ||
               (geneticEvaluatorPtr->dp.ground_truth_path != dp.ground_truth_path)) {
        geneticEvaluatorPtr->initialise_datapoint(dp);
    }

    for (auto &alg:hv_algorithms) {
        alg->update_variables();
        if (alg->enable) {
            alg->run(geneticEvaluatorPtr,rawDataMapAlgObjVec[alg->name][obj]);
        }
    }
}

std::vector<double> AlgorithmTuner::get_param_test_values() {
    std::vector<double> param_test_values;

    double paramTestMin = paramMinDoubleSpinBox->value();
    double paramTestMax = paramMaxDoubleSpinBox->value();
    double paramTestStep = paramStepDoubleSpinBox->value();

    param_test_values.resize(static_cast<int>((paramTestMax - paramTestMin) / paramTestStep)+1);
    std::iota(param_test_values.begin(), param_test_values.end(), 0); // Generate step indices
    std::transform(param_test_values.begin(), param_test_values.end(), param_test_values.begin(),[paramTestMin,paramTestStep](double &c) {return paramTestMin+c * paramTestStep;}); // multiply each step with step index length
    return param_test_values;
}


void AlgorithmTuner::run_enabled_algorithms() {
    progressBar->show();
    algorithmDataProcs.clear();
    rawData.clear();
    if (scapeDatasetPtr != nullptr) {
        std::vector<double> param_test_values;
        if(paramComboBox->count()>0)
            param_test_values = get_param_test_values();
        double initial_param_test_value = getSpinBoxWidgetValue(parameter_test_widget);

        std::vector<std::pair<DatasetObjectPtr,std::vector<DataPoint>>> object_and_datapoint_pairs;
        if (runEnabledMethodsComboBox->currentIndex() == 0){
            DatasetObjectPtr obj = scapeDatasetPtr->get_object_by_name(objectNameComboBox->currentText().toStdString());
            object_and_datapoint_pairs.emplace_back(obj,std::vector{obj->data_points[datapointSpinBox->value()]});
        }else if (runEnabledMethodsComboBox->currentIndex() == 1) {
            DatasetObjectPtr obj = scapeDatasetPtr->get_object_by_name(objectNameComboBox->currentText().toStdString());
            object_and_datapoint_pairs.emplace_back(obj,std::vector<DataPoint>{});
            for (auto &dp:obj->data_points) {
                if (dp.gts.size() > 1)
                    object_and_datapoint_pairs.back().second.emplace_back(dp);
            }
        }
        else if (runEnabledMethodsComboBox->currentIndex() == 2) {
            for (auto &obj:scapeDatasetPtr->objects) {
                object_and_datapoint_pairs.emplace_back(obj,std::vector<DataPoint>{});
                for (auto &dp:obj->data_points) {
                    if (dp.gts.size() > 1) {
                        object_and_datapoint_pairs.back().second.emplace_back(dp);
                    }
                }
            }
        }




        EvaluatorInterfacePtr evaluatorInterfacePtr = get_evaluator_interface(evaluator_types_combo_box->currentText());
        evaluatorInterfacePtr->update_variables();
        GeneticEvaluatorPtr &geneticEvaluatorPtr = evaluatorInterfacePtr->geneticEvaluatorPtr;

        int n_dp = 0, tot_n_dp = 0;
        for(auto &obj_dp_pair:object_and_datapoint_pairs) {
            tot_n_dp += obj_dp_pair.second.size();
        }
        progressBar->setMaximum(tot_n_dp);
        progressBar->show();


        set_data_info_doc_alg_var_names();
        rawData.clear();
        if(paramComboBox->count()>0) {
            bool first_run = true;
            rawData.resize(param_test_values.size());
            for (auto &obj_dp_pair:object_and_datapoint_pairs) {
                for (auto &dp:obj_dp_pair.second) {
                    for (int i = 0;i<param_test_values.size();i++) {
                        auto &val = param_test_values[i];
                        setSpinBoxWidgetValue(parameter_test_widget, val);
                        evaluatorInterfacePtr->update_variables();
                        run_enabled_algorithms(geneticEvaluatorPtr, obj_dp_pair.first, dp, rawData[i]);
                        progressBar->setValue(++n_dp/param_test_values.size());
                        QCoreApplication::processEvents();
                        if(first_run)
                            set_data_info_doc_alg_variables(i);
                    }
                    if(first_run)
                        first_run = false;
                }
            }
            setSpinBoxWidgetValue(parameter_test_widget, initial_param_test_value);
        }
        else{
            rawData.emplace_back();
            for (auto &obj_dp_pair:object_and_datapoint_pairs) {
                for (auto &dp:obj_dp_pair.second) {
                    evaluatorInterfacePtr->update_variables();
                    run_enabled_algorithms(geneticEvaluatorPtr, obj_dp_pair.first, dp, rawData.back());
                    progressBar->setValue(++n_dp);
                    QCoreApplication::processEvents();
                    set_data_info_doc_alg_variables(0);
                }
            }
        }


        progressBar->hide();

        rawDataMapAlgObjVecT& rawDataMapAlgObjVec = rawData[0];

        for(auto &d:rawData)
            algorithmDataProcs.emplace_back(d, general_settings.ground_truth_t_thresh->value(),general_settings.ground_truth_r_thresh->value());

        dataProcSaveResultsButton->setEnabled(true);


        // Clear and update visualizer
        group_vis->clear();
        group_vis->addIdPointCloud(geneticEvaluatorPtr->pc, "Captured Point Cloud");
        // Add gt and data to visualization
        auto obj = scapeDatasetPtr->get_object_by_name(algorithmDataProcs[0].objName.back());
        auto &dp = rawDataMapAlgObjVec.begin()->second[obj].back().dp;
        PointCloudT::Ptr meshpc = obj->get_mesh_point_cloud();

        auto var_itt = std::find_if(evaluatorInterfacePtr->variables_d.begin(),evaluatorInterfacePtr->variables_d.end(),[](var_d &var){return var.name == "VoxelGrid leaf size";});//
        if(var_itt != evaluatorInterfacePtr->variables_d.end()) {
            GeneticEvaluatorOCPtr geneticEvaluatorOcPtr = std::dynamic_pointer_cast<GeneticEvaluatorOC>(geneticEvaluatorPtr);
            for (int g = 0; g < dp.gts.size(); g++) {
                PointCloudT::Ptr gtpc_vis = pcl::make_shared<PointCloudT>();
                std::string id = "gt_" + std::to_string(g);
                pcl::transformPointCloud(*meshpc, *gtpc_vis, dp.gts[g]);
                geneticEvaluatorOcPtr->voxelGridPtr->setInputCloud(gtpc_vis);
                geneticEvaluatorOcPtr->voxelGridPtr->setLeafSize(*(var_itt->val),*(var_itt->val),*(var_itt->val));
                geneticEvaluatorOcPtr->voxelGridPtr->filter(*gtpc_vis);
                group_vis->addIdPointCloud(gtpc_vis, id, "Ground Truth", 0, 255, 255);
            }
        }else{
            for (int g = 0; g < dp.gts.size(); g++) {
                PointCloudT::Ptr gtpc_vis = pcl::make_shared<PointCloudT>();
                std::string id = "gt_" + std::to_string(g);
                pcl::transformPointCloud(*meshpc, *gtpc_vis, dp.gts[g]);
                group_vis->addIdPointCloud(gtpc_vis, id, "Ground Truth", 0, 255, 255);
            }
        }
        group_vis->toggle_group_opacity(group_vis->find_pcv_group_id("Ground Truth"));

        for (auto &alg:hv_algorithms) {
            if (alg->enable) {
                size_t bi, ei;
                algorithmDataProcs[0].get_begin_end_index(bi, ei, alg->name, obj->name);
                add_results_to_visualizer(geneticEvaluatorPtr, *(algorithmDataProcs[0].algName.begin() + ei - 1),*(algorithmDataProcs[0].algName.begin() + ei - 1), *(algorithmDataProcs[0].tpIVec.begin() + ei - 1), *(algorithmDataProcs[0].tnIVec.begin() + ei - 1), *(algorithmDataProcs[0].fpIVec.begin() + ei - 1), *(algorithmDataProcs[0].fnIVec.begin() + ei - 1));
            }
        }
        group_vis->resetCamera();
        group_vis->update_text();

        // Generate Result Table
        std::vector<std::string> dataTypes = {"Acc", "Rec", "Pre"};
        resultTableWidget->clear();
        resultTableWidget->setColumnCount(algorithmDataProcs[0].uniqAlgNames.size());
        resultTableWidget->setRowCount(dataTypes.size());

        QStringList horizontalHeaders, verticalHeaders;
        std::transform(algorithmDataProcs[0].uniqAlgNames.begin(), algorithmDataProcs[0].uniqAlgNames.end(),
                       std::back_inserter(horizontalHeaders), [](std::string &s) { return QString::fromStdString(s); });
        std::transform(dataTypes.begin(), dataTypes.end(), std::back_inserter(verticalHeaders),
                       [](std::string &s) { return QString::fromStdString(s); });
        resultTableWidget->setHorizontalHeaderLabels(horizontalHeaders);
        resultTableWidget->setVerticalHeaderLabels(verticalHeaders);

        int col_n = 0;
        for (auto &alg:algorithmDataProcs[0].uniqAlgNames) {
            resultTableWidget->setCellWidget(0, col_n, new QLabel(QString::fromStdString(
                    std::to_string(algorithmDataProcs[0].get_avr(algorithmDataProcs[0].accuracy, alg)))));
            resultTableWidget->setCellWidget(1, col_n, new QLabel(QString::fromStdString(
                    std::to_string(algorithmDataProcs[0].get_avr(algorithmDataProcs[0].recall, alg)))));
            resultTableWidget->setCellWidget(2, col_n, new QLabel(QString::fromStdString(
                    std::to_string(algorithmDataProcs[0].get_avr(algorithmDataProcs[0].precision, alg)))));
            col_n++;
        }

        plotComboBox->clear();
        auto names = algorithmDataProcs[0].derivedCSVDocPtr->GetColumnNames();
        for(int i = 0;i<names.size();i++)
            if(i>3)
                plotComboBox->addItem(QString::fromStdString(names[i]));

        dataProcDock->show();
        resultDock->show();
    }
}


void AlgorithmTuner::add_results_to_visualizer(GeneticEvaluatorPtr &geneticEvaluatorPtr, std::string group,
                                               std::string node_prefix, std::vector<int> tp, std::vector<int> tn,
                                               std::vector<int> fp, std::vector<int> fn) {

    if(geneticEvaluatorPtr->type == "GeneticEvaluatorOC") {
        auto geneticEvaluatorOCPtr = std::dynamic_pointer_cast<GeneticEvaluatorOC>(geneticEvaluatorPtr);

        for (int i = 0; i < geneticEvaluatorOCPtr->dp.ocs.size(); i++) {
            std::string id = node_prefix + "_oc_" + std::to_string(i);

            if (std::find(tp.begin(), tp.end(), i) != tp.end()) {
                group_vis->addIdPointCloud(geneticEvaluatorOCPtr->visible_oc_pcs[i], id,
                                           group + "/True Positives", 0, 255, 0);
            } else if (std::find(fp.begin(), fp.end(), i) != fp.end()) {
                group_vis->addIdPointCloud(geneticEvaluatorOCPtr->visible_oc_pcs[i], id,
                                           group + "/False Positives", 255, 0, 0);
            } else if (std::find(tn.begin(), tn.end(), i) != tn.end()) {
                group_vis->addIdPointCloud(geneticEvaluatorOCPtr->visible_oc_pcs[i], id,
                                           group + "/True Negatives", 255, 255, 0);
            } else if (std::find(fn.begin(), fn.end(), i) != fn.end()) {
                group_vis->addIdPointCloud(geneticEvaluatorOCPtr->visible_oc_pcs[i], id,
                                           group + "/False Negatives", 0, 0, 255);
            }
        }
    }
}


void AlgorithmTuner::timeoutSlot() {
    vtkWidget->update();
}

void AlgorithmTuner::update_evaluator_type(const QString &s) {
    while(evaluatorSettingsFormLayout->count())
        evaluatorSettingsFormLayout->removeRow(0);

    auto eval = get_evaluator_interface(s);
    if(eval!= nullptr) {
        eval->add_variable_to_formlayout(evaluatorSettingsFormLayout);
        current_evaluator_text = s;
    }
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

    std::string time_string  = getTimeString("%Y-%d-%m-%H-%M-%S");
    QString proposed_file_name = QString::fromStdString((last_save_folder / (time_string + "-AlgorithmTunerData.csv")).string());
    settings.data_save_file = QFileDialog::getSaveFileName(this, "Save Dynamic Data", proposed_file_name);

    auto path = std::filesystem::path(settings.data_save_file.toStdString());
    auto filename = std::filesystem::path(settings.data_save_file.toStdString()).filename().replace_extension();
    auto parent_folder = std::filesystem::path(settings.data_save_file.toStdString()).parent_path();

    for(int i = 0;i<algorithmDataProcs.size();i++){
        std::string fn = path.replace_filename(filename.string()+"_"+std::to_string(i)+".csv");
        algorithmDataProcs[i].derivedCSVDocPtr->Save(fn);
        data_info_doc->SetCell(0,i,fn);
    }

    data_info_doc->Save(path.replace_filename(time_string+"-data_info.csv"));
    //rapidcsv::CSVWriteDoc data_info( path.replace_filename(time_string+"-data_info"),rapidcsv::LabelParams(0, -1));
}

void AlgorithmTuner::bar_plot() {
    std::string plotDataType = plotComboBox->currentText().toStdString();

    auto f = algorithmDataProcs[0].bar_plot(plotDataType);
    f->current_axes()->ylabel(plotDataType);
    f->draw();
}

EvaluatorInterfacePtr AlgorithmTuner::get_evaluator_interface(QString name) {
    auto eval_itt = std::find_if(evaluators.begin(),evaluators.end(),[name](EvaluatorInterfacePtr &evaluatorInterfacePtr){return evaluatorInterfacePtr->name == name.toStdString();});
    if(eval_itt!=evaluators.end())
        return *eval_itt;
    else
        return nullptr;
}

void AlgorithmTuner::update_range_params(const QString &s) {
    paramComboBox->clear();

    auto it = std::find_if(algorithms.begin(),algorithms.end(),[&s](const AlgorithmInterfacePtr &algIPtr){return (*algIPtr) == s.toStdString();});

    if(it != algorithms.end())
    {
        for(auto& param:(*it)->variables_i)
            paramComboBox->addItem(QString::fromStdString(param.name));
        for(auto& param:(*it)->variables_d)
            paramComboBox->addItem(QString::fromStdString(param.name));
    }
}

void AlgorithmTuner::update_range_param_limits(const QString &s) {

    auto it = std::find_if(algorithms.begin(),algorithms.end(),[this](const AlgorithmInterfacePtr &algIPtr){return (*algIPtr) == algComboBox->currentText().toStdString();});
    if(it != algorithms.end()){
        auto it_i = std::find_if((*it)->variables_i.begin(),(*it)->variables_i.end(),[s](const var_i &vi){return vi.name == s.toStdString();});
        auto it_d = std::find_if((*it)->variables_d.begin(),(*it)->variables_d.end(),[s](const var_d &vd){return vd.name == s.toStdString();});

        if(it_i != (*it)->variables_i.end()) {
            paramMinDoubleSpinBox->setDecimals(0);
            paramMaxDoubleSpinBox->setDecimals(0);
            paramStepDoubleSpinBox->setDecimals(0);
            parameter_test_widget = it_i->spinBox;
            parameter_test_name = it_i->name;
        }
        if(it_d != (*it)->variables_d.end()) {
            paramMinDoubleSpinBox->setDecimals(3);
            paramMaxDoubleSpinBox->setDecimals(3);
            paramStepDoubleSpinBox->setDecimals(3);
            parameter_test_widget = it_d->spinBox;
            parameter_test_name = it_d->name;
        }
    }
}

void AlgorithmTuner::param_test_plot() {

    QHBoxLayout* qhBoxLayout = new QHBoxLayout;
    QFormLayout* qFormLayout_dt = new QFormLayout;
    QFormLayout* qFormLayout_alg = new QFormLayout;
    std::vector<QCheckBox*> checkBoxes_dt;
    std::vector<QCheckBox*> checkBoxes_alg;

    for(int i = 0;i<plotComboBox->count();i++){
        checkBoxes_dt.emplace_back(new QCheckBox());
        qFormLayout_dt->addRow(new QLabel(plotComboBox->itemText(i)), checkBoxes_dt.back());
    }

    for(auto& algName: algorithmDataProcs[0].uniqAlgNames) {
        checkBoxes_alg.emplace_back(new QCheckBox());
        qFormLayout_alg->addRow(new QLabel(QString::fromStdString(algName)), checkBoxes_alg.back());
    }

    qhBoxLayout->addLayout(qFormLayout_alg);
    qhBoxLayout->addLayout(qFormLayout_dt);

    QDialog* qDialog = new QDialog(this);
    qDialog->setLayout(qhBoxLayout);
    qDialog->exec();

    std::vector<std::string> data_types,algs;
    for(int i = 0; i < checkBoxes_dt.size(); i++) {
        if (checkBoxes_dt[i]->isChecked()) {
            data_types.emplace_back(qobject_cast<QLabel*>(qFormLayout_dt->labelForField(checkBoxes_dt[i]))->text().toStdString());
        }
    }
    for(int i = 0; i < checkBoxes_alg.size(); i++) {
        if (checkBoxes_alg[i]->isChecked()) {
            algs.emplace_back(qobject_cast<QLabel*>(qFormLayout_alg->labelForField(checkBoxes_alg[i]))->text().toStdString());
        }
    }

    std::vector<std::vector<std::vector<double>>> data(data_types.size());
    for(auto&v:data)
        v.resize(algs.size());


    for(auto& dataProc:algorithmDataProcs) {
        for (int dt = 0; dt < data_types.size(); dt++) {
            auto dataVec = dataProc.derivedCSVDocPtr->GetColumn<double>(data_types[dt]);
            for(int a = 0;a<algs.size();a++) {
                data[dt][a].emplace_back(dataProc.get_avr(dataVec, algs[a]));
            }
        }
    }

    std::vector<std::string> legends;
    for (auto& dt : data_types) {
        for(auto&a : algs){
            legends.emplace_back(a+" "+dt);
        }
    }

    using namespace matplot;
    auto f = figure(true);
    auto ax = f->current_axes();

    std::vector<double> param_test_values = get_param_test_values();

    hold(on);
    for(auto&dt:data) {
        for(auto&ad:dt) {
            ax->plot(param_test_values,ad);
        }
    }
    hold(off);

    ax->xlabel(parameter_test_name);
    legend(ax,legends);

    f->draw();

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

void AlgorithmTuner::set_data_info_doc_alg_var_names() {
    data_info_doc->clear();
    data_info_doc->SetColumnName(0,"data_path");
    data_info_doc->SetColumnName(1,"t_thresh");
    data_info_doc->SetColumnName(2,"r_thresh");
    int col_name_idx = 3;

    std::string current_eval_str = evaluator_types_combo_box->currentText().toStdString();
    auto eval_it = std::find_if(evaluators.begin(),evaluators.end(),[current_eval_str](EvaluatorInterfacePtr &evaluatorInterfacePtr){return evaluatorInterfacePtr->name == current_eval_str;});
    if(eval_it!=evaluators.end()){
        auto& eval = *eval_it;
        eval->update_variables();
        for (auto vi:eval->variables_i)
            data_info_doc->SetColumnName(col_name_idx++, eval->name + "/" + vi.name);
        for (auto vd:eval->variables_d)
            data_info_doc->SetColumnName(col_name_idx++, eval->name + "/" + vd.name);
        for (auto vb:eval->variables_b)
            data_info_doc->SetColumnName(col_name_idx++, eval->name + "/" + vb.name);
    }
    for(auto alg:hv_algorithms) {
        alg->update_variables();
        if(alg->enable) {
            for (auto vi:alg->variables_i)
                data_info_doc->SetColumnName(col_name_idx++, alg->name + "/" + vi.name);
            for (auto vd:alg->variables_d)
                data_info_doc->SetColumnName(col_name_idx++, alg->name + "/" + vd.name);
            for (auto vb:alg->variables_b)
                data_info_doc->SetColumnName(col_name_idx++, alg->name + "/" + vb.name);
        }
    }
}

void AlgorithmTuner::set_data_info_doc_alg_variables(int row_i) {


    data_info_doc->SetCell(1,row_i,general_settings.ground_truth_t_thresh->value());
    data_info_doc->SetCell(2,row_i,general_settings.ground_truth_r_thresh->value());

    std::string current_eval_str = evaluator_types_combo_box->currentText().toStdString();
    auto eval_it = std::find_if(evaluators.begin(),evaluators.end(),[current_eval_str](EvaluatorInterfacePtr &evaluatorInterfacePtr){return evaluatorInterfacePtr->name == current_eval_str;});
    if(eval_it!=evaluators.end()){
        auto& eval = *eval_it;
        eval->update_variables();
        for (auto vi:eval->variables_i)
            data_info_doc->SetCell(data_info_doc->GetColumnIdx( eval->name + "/" + vi.name),row_i,*vi.val);
        for (auto vd:eval->variables_d)
            data_info_doc->SetCell(data_info_doc->GetColumnIdx( eval->name + "/" + vd.name),row_i,*vd.val);
        for (auto vb:eval->variables_b)
            data_info_doc->SetCell(data_info_doc->GetColumnIdx( eval->name + "/" + vb.name),row_i,*vb.val);
    }
    for(auto alg:hv_algorithms) {
        alg->update_variables();
        if(alg->enable) {
            for (auto vi:alg->variables_i)
                data_info_doc->SetCell(data_info_doc->GetColumnIdx( alg->name + "/" + vi.name),row_i,*vi.val);
            for (auto vd:alg->variables_d)
                data_info_doc->SetCell(data_info_doc->GetColumnIdx( alg->name + "/" + vd.name),row_i,*vd.val);
            for (auto vb:alg->variables_b)
                data_info_doc->SetCell(data_info_doc->GetColumnIdx( alg->name + "/" + vb.name),row_i,*vb.val);
        }
    }
}

