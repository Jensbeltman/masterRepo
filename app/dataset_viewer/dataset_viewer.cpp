#include "dataset_viewer.hpp"

#include <QEvent>
#include <QDialog>
#include <QObject>
#include <QFileDialog>
#include <QTableWidget>
#include <iostream>
#include <dataset/scape/ScapeDataset.hpp>
#include "manual_registration.h"

DatasetViewer::DatasetViewer(QMainWindow *parent) : QMainWindow(parent) {
    setupUi(this);
    QObject::connect(actionOpen, &QAction::triggered, this, &DatasetViewer::chose_dataset_folders);
    QObject::connect(tableWidget, &QTableWidget::cellDoubleClicked, this, &DatasetViewer::open_anotator);

    mr = new ManualRegistration(this); // Initialize first ManualRegistration Window
    loadSettings(); // Load setting from prev session e.g. paths for dataset
}

void DatasetViewer::changeEvent(QEvent *e) {
    QWidget::changeEvent(e);
    switch (e->type()) {
        case QEvent::LanguageChange:
            retranslateUi(this);
            break;
        default:
            break;
    }
}

void DatasetViewer::chose_dataset_folders() {
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

void DatasetViewer::loadSettings() {
    QSettings qsettings;
    settings.data_folder = qsettings.value("data_folder", "/home").toString();
    settings.recognition_folder = qsettings.value("recognition_folder", "/home").toString();
}

void DatasetViewer::saveSettings() {
    QSettings qsettings;
    qsettings.setValue("data_folder", settings.data_folder);
    qsettings.setValue("recognition_folder", settings.recognition_folder);
}

DatasetViewer::~DatasetViewer() {
    saveSettings();
    delete this;
}


void DatasetViewer::load_dataset() {
    scapeDatasetPtr = std::make_shared<ScapeDataset>(settings.data_folder.toStdString(),
                                                                     settings.recognition_folder.toStdString(), true);
    datasetTitle->setText((scapeDatasetPtr->name + " Dataset").c_str());
    tableWidget->setRowCount(0);
    obj_ns.clear();

    int current_row;
    for (auto obj : scapeDatasetPtr->objects) {
        auto sobj =  std::static_pointer_cast<ScapeDatasetObject>(obj);
        for (auto &fn : obj->pcd_filenames) {
            auto &dpis = sobj->pcd_fn_to_scape_dpis[fn];
            if(!dpis.empty()){
                obj_ns.emplace_back(dpis);
                auto &dp = obj->data_points[dpis[0]];
                tableWidget->insertRow(tableWidget->rowCount());

                current_row = tableWidget->rowCount() - 1;
                tableWidget->setItem(current_row, 0, new QTableWidgetItem(QString(obj->name.c_str())));
                tableWidget->setItem(current_row, 1, new QTableWidgetItem(QString(dp.pcd_filename.c_str())));

                tableWidget->setItem(current_row, 2, new QTableWidgetItem(
                        QString(std::to_string(dp.gts.size()).c_str())));

                std::string oc_string="";
                for(int i = 0;i<dpis.size()-1;i++)
                    oc_string+=std::to_string( obj->data_points[dpis[i]].ocs.size())+", ";
                oc_string+=std::to_string(obj->data_points[dpis.back()].ocs.size());

                tableWidget->setItem(current_row, 3, new QTableWidgetItem(
                        QString(oc_string.c_str())));
            }
        }
    }
}

std::ostream& operator << (std::ostream& os, std::vector<int> &ints)
{
    for(int i = 0;i<ints.size()-1;i++)
        os<<ints[i]<<", ";
    os<<ints.back();
    return os;
}

void DatasetViewer::open_anotator(int row, int column) {
    if (mr->isVisible()){
        std::cout<<"A Manual Registration is still in the process please close it"<<std::endl;
    }else{
        delete mr;
        mr = new ManualRegistration(this);

        std::string fn = tableWidget->item(row,1)->text().toStdString();
        ScapeDatasetObjectPtr datasetObjPtr = scapeDatasetPtr->get_scape_object_by_name(tableWidget->item(row,0)->text().toStdString());

        auto &dp = datasetObjPtr->data_points[obj_ns[row][0]];
        mr->setSrcCloud(datasetObjPtr->get_mesh_point_cloud(),datasetObjPtr->mesh_path);
        mr->setDstCloud(datasetObjPtr->get_pcd(obj_ns[row][0])); // Chose pcd based on first dp as they all should have the same
        if(dp.ground_truth_path.empty()) {
            std::filesystem::path p = datasetObjPtr->path;
            p = p.parent_path();
            p/="gt";
            p/=(datasetObjPtr->name_singular+"-"+dp.pcd_filename+".txt");
            mr->setGTs(dp.gts,p.string());
        }
        else {
            mr->setGTs(dp.gts, dp.ground_truth_path);
        }

        if(obj_ns[row].size()>1)
        {
            int total_ocs=0;
            std::vector<T4> all_ocs;
            for(auto &on:obj_ns[row]) {
                total_ocs+=(datasetObjPtr->data_points[on].ocs.size());
                for(auto &t4:datasetObjPtr->data_points[on].ocs){
                    auto itt = all_ocs.begin();
                    for(;itt!=all_ocs.end();++itt){
                        if(itt->isApprox(t4))
                            break;
                    }
                    if(itt==all_ocs.end()){
                        all_ocs.emplace_back(t4);
                    }
                }
            }
            std::cout<<"A toal of "<<total_ocs<<" ocs found in multiple files, merged to "<<all_ocs.size()<<std::endl;
            mr->setOCs(all_ocs);
        }else{
            mr->setOCs(dp.ocs);
        }



        mr->show();

        mr->setup();

        std::cout<<"Starting anotation of data points "<<obj_ns[row]<<" from object "<<tableWidget->item(row,0)->text().toStdString()<<"\n";
        std::cout << "PCD file: " << dp.pcd_filename << "\nGround Truth path: " << dp.ground_truth_path << std::endl;
    }
}
