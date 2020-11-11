#include "dataset_viewer.hpp"

#include <QEvent>
#include <QDialog>
#include <QObject>
#include <QFileDialog>
#include <QTableWidget>
#include <iostream>
#include <dataset/scape/ScapeDataset.hpp>

DatasetViewer::DatasetViewer(QMainWindow *parent):QMainWindow(parent) {
    setupUi(this);
    QObject::connect(actionOpen,&QAction::triggered,this, &DatasetViewer::chose_dataset_folders);
    loadSettings();
    filesystemModel = new QFileSystemModel;
}

void DatasetViewer::changeEvent(QEvent *e)
{
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
    QString data_folder = QFileDialog::getExistingDirectory(this,"Select data folder",settings.data_folder);
    QString recognition_folder = QFileDialog::getExistingDirectory(this,"Select recognition folder",settings.recognition_folder);

    if(!data_folder.isNull())
        settings.data_folder = data_folder;
    if(!recognition_folder.isNull())
        settings.recognition_folder = recognition_folder;

    std::cout<<"data folder: "<<data_folder.toStdString()<<", recognition folder: "<<recognition_folder.toStdString()<<std::endl;

    load_dataset();

}

void DatasetViewer::loadSettings() {
    QSettings qsettings;
    settings.data_folder = qsettings.value("data_folder", "/home").toString();
    settings.recognition_folder = qsettings.value("recognition_folder", "/home").toString();

}

void DatasetViewer::saveSettings() {
    QSettings qsettings;
    qsettings.setValue("data_folder",settings.data_folder);
    qsettings.setValue("recognition_folder",settings.recognition_folder);
}

DatasetViewer::~DatasetViewer() {
    saveSettings();
    delete this;
}



void DatasetViewer::load_dataset() {
    ScapeDatasetPtr scapeDatasetPtr =  std::make_shared<ScapeDataset>(settings.data_folder.toStdString(),settings.recognition_folder.toStdString(),true);

    tableWidget->setRowCount(0);

for(auto obj:scapeDatasetPtr->objects) {
    ScapeDatasetObjectPtr sobj = std::static_pointer_cast<ScapeDatasetObject>(obj);
    int prev_row_count = tableWidget->rowCount();
    tableWidget->setRowCount(prev_row_count + obj->filenames.size());
    for (int r = 0; r < obj->filenames.size(); r++) {
        tableWidget->setItem(prev_row_count+r, 0, new QTableWidgetItem(QString(obj->name.c_str())));
        tableWidget->setItem(prev_row_count+r, 1, new QTableWidgetItem(QString(obj->filenames[r].c_str())));
        if(!sobj->fn_to_zone[obj->filenames[r]].empty()){
            tableWidget->setItem(prev_row_count + r, 2, new QTableWidgetItem(
                    QString(std::to_string((obj->get_gt(sobj->fn_to_zone[obj->filenames[r]][0]).size())).c_str())));
            tableWidget->setItem(prev_row_count + r, 3, new QTableWidgetItem(
                    QString(std::to_string((sobj->zones[sobj->fn_to_zone[obj->filenames[r]][0]].ocs.size())).c_str())));
        }else{
            tableWidget->setItem(prev_row_count + r, 2, new QTableWidgetItem(
                    QString("0")));
            tableWidget->setItem(prev_row_count + r, 3, new QTableWidgetItem(
                    QString("0")));
        }
    }
}
}
