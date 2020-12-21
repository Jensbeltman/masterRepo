#include <QtWidgets/QApplication>
#include "algorithm_tuner.hpp"
#include <QDockWidget>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setOrganizationName("University of Southern Denmark");
    app.setOrganizationDomain("SDU");
    app.setApplicationName("AlgorithmTuner");
    QMainWindow *mainWindow = new QMainWindow;
    AlgorithmTuner algorithmTuner(mainWindow);
    algorithmTuner.show();
    return app.exec();
}