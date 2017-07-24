#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "freespaceboundary.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void onUpdateAll();
    void on_pb_setcontact_clicked();

    void on_pb_savetofile_clicked();

    void on_pb_loadfromfile_clicked();

    void on_pb_saveimage_clicked();

signals:
    void updateAll();

private:
    Ui::MainWindow *ui;
    FreeSpaceBoundary bfp;
};

#endif // MAINWINDOW_H
