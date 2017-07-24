#include "mainwindow.h"
#include <QApplication>

#include <QSurfaceFormat>

int main(int argc, char *argv[])
{
    QSurfaceFormat format;
    format.setVersion(3,3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(8);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
