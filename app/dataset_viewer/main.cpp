#include <QtWidgets/QApplication>
#include "dataset_viewer.hpp"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setOrganizationName("University of Southern Denmark");
    app.setOrganizationDomain("SDU");
    app.setApplicationName("DatasetViewer");
    QMainWindow *mainWindow = new QMainWindow;
    DatasetViewer datasetViewer(mainWindow);
    datasetViewer.show();
    return app.exec();
}