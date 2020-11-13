#ifndef MASTER_DATASET_VIEWER_HPP
#define MASTER_DATASET_VIEWER_HPP

#include <ui_dataset_viewer.h>

#include <QMainWindow>
#include <QMutex>
#include <QTimer>
#include <QSettings>
#include <QFileSystemModel>
#include <dataset/scape/ScapeDataset.hpp>
#include "manual_registration.h"

struct Settings {
    QString data_folder;
    QString recognition_folder;
};

namespace Ui {
    class DatasetViewer;
}

class DatasetViewer : public QMainWindow, private Ui::DatasetViewer {
Q_OBJECT
public:
    explicit DatasetViewer(QMainWindow *parent = nullptr);

    ~DatasetViewer();

    void changeEvent(QEvent *e);

private:
    Settings settings;
    ManualRegistration *mr;
    ScapeDatasetPtr scapeDatasetPtr;
    std::vector<std::vector<int>> obj_ns;

    void loadSettings();

    void saveSettings();

    void load_dataset();

private slots:

    void chose_dataset_folders();

    void open_anotator(int row, int column);

};


#endif //MASTER_DATASET_VIEWER_HPP
