#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    //Q_INIT_RESOURCE(application);

    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName("Lucas Placentino");
    QCoreApplication::setApplicationName("5G channel modeling raytracing");
    QCoreApplication::setApplicationVersion(QT_VERSION_STR);

    MainWindow mainWin;
    mainWin.show();

    return app.exec();
}

