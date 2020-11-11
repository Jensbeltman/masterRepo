#ifndef MASTER_DATASET_VIEWER_HPP
#define MASTER_DATASET_VIEWER_HPP

#include <ui_dataset_viewer.h>

#include <QMainWindow>
#include <QMutex>
#include <QTimer>
#include <QSettings>
#include <QFileSystemModel>
#include <dataset/Dataset.hpp>

struct Settings{
    QString data_folder;
    QString recognition_folder;
};

namespace Ui {
    class DatasetViewer;
}

class DatasetViewer: public QMainWindow, private Ui::DatasetViewer
{
    Q_OBJECT
public:
    explicit DatasetViewer(QMainWindow *parent = nullptr);
    ~DatasetViewer();
    void changeEvent(QEvent *e);

private:
    void loadSettings();
    void saveSettings();

    Settings settings;
    DatasetPtr datasetPtr;
    QFileSystemModel *filesystemModel;
    void load_dataset();
private slots:
    void chose_dataset_folders();



};


#endif //MASTER_DATASET_VIEWER_HPP
