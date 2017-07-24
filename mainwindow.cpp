#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    findChild<StateGLWidget*>()->attachScene(&bfp);
    findChild<GraphEditor*>("objEditor")->setGraph(&(bfp.object()));
    findChild<GraphEditor*>("obsEditor")->setGraph(&(bfp.obstacles()));
    findChild<ThetaSliceView*>()->attachScene(&bfp);
    findChild<ContactWidget*>("contactView1")->attachScene(&bfp);
    findChild<ContactWidget*>("contactView2")->attachScene(&bfp);
    findChild<SceneWidget*>("scene")->attachScene(&bfp);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onUpdateAll()
{
    emit updateAll();
}

void MainWindow::on_pb_setcontact_clicked()
{
    int topV = findChild<QSpinBox*>("sb_topV")->value();
    int topE = findChild<QSpinBox*>("sb_topE")->value();
    int botV = findChild<QSpinBox*>("sb_botV")->value();
    int botE = findChild<QSpinBox*>("sb_botE")->value();
    Contact::ContactType type =
            Contact::NoContact;
    if(findChild<QRadioButton*>("rb_VE")->isChecked()){
        type = Contact::VertexEdge;
    }
    if(findChild<QRadioButton*>("rb_EV")->isChecked()){
        type = Contact::EdgeVertex;
    }
    if(type == Contact::NoContact)return;

    findChild<ContactWidget*>("contactView1")->setContact(topE,topV,type);
    findChild<ContactWidget*>("contactView2")->setContact(botE,botV,type);
}
#include <QFileDialog>
void MainWindow::on_pb_savetofile_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,"Save File");
    QFile f(filename);
    f.open(QIODevice::WriteOnly);
    QDataStream out(&f);
    out << bfp;
    f.close();
}

void MainWindow::on_pb_loadfromfile_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,"Load File");
    QFile f(filename);
    if(!(f.open(QIODevice::ReadOnly))){
        return;
    }
    QDataStream in(&f);
    in >> bfp;
    f.close();
    emit updateAll();
}

void MainWindow::on_pb_saveimage_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,"Save Image");
    QPixmap pixmap(findChild<ContactWidget*>("contactView2")->size());
    findChild<ContactWidget*>("contactView2")->render(&pixmap);
    pixmap.save(filename);
}
