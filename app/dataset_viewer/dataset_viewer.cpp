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
    mr = new ManualRegistration(this);
    loadSettings();
    filesystemModel = new QFileSystemModel;
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
    int glob_count;
    for (auto obj:scapeDatasetPtr->objects) {
        int prev_row_count = tableWidget->rowCount();
        tableWidget->setRowCount(prev_row_count + obj->size());
        obj_ns.resize(prev_row_count + obj->size());
        for (int r = 0; r < obj->size(); r++) {
            auto &dp = obj->data_points[r];
            glob_count=prev_row_count + r;
            obj_ns[glob_count] = r;
            tableWidget->setItem(glob_count, 0, new QTableWidgetItem(QString(obj->name.c_str())));
            tableWidget->setItem(glob_count, 1, new QTableWidgetItem(QString(dp.pcd_filename.c_str())));

            tableWidget->setItem(glob_count, 2, new QTableWidgetItem(
                    QString(std::to_string(dp.gts.size()).c_str())));
            tableWidget->setItem(glob_count, 3, new QTableWidgetItem(
                    QString(std::to_string(dp.ocs.size()).c_str())));
        }
    }
}

void DatasetViewer::open_anotator(int row, int column) {

    if (mr->isVisible()){
        std::cout<<"A Manual Registration is still in the process please close it"<<std::endl;
    }else{
        std::cout<<"Starting anotation of data point "<<obj_ns[row]<<" from object "<<tableWidget->itemAt(0,row)->text().toStdString()<<"\n";
        delete mr;
        mr = new ManualRegistration(this);
        mr->setDstCloud(scapeDatasetPtr->get_object_by_name(tableWidget->itemAt(0,row)->text().toStdString())->get_pcd(obj_ns[row]));
        mr->setSrcCloud(scapeDatasetPtr->get_object_by_name(tableWidget->itemAt(0,row)->text().toStdString())->get_mesh_point_cloud());
        mr->show();
    }
}
