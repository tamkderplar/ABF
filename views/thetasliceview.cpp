#include "thetasliceview.h"
#include "ui_thetasliceview.h"

ThetaSliceView::ThetaSliceView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ThetaSliceView)
{
    ui->setupUi(this);
    auto colorNames = QColor::colorNames();
    for(auto colorName: colorNames){
        QPixmap pixel(findChild<QComboBox*>()->iconSize());
        pixel.fill(QColor(colorName));
        QIcon icon(pixel);
        findChild<QComboBox*>()->insertItem(0,icon,colorName);
        //findChild<QComboBox*>()->setItemData(0,QColor(colorName),Qt::DecorationRole);
    }
}

ThetaSliceView::~ThetaSliceView()
{
    delete ui;
}

void ThetaSliceView::attachScene(FreeSpaceBoundary *s)
{
    findChild<ThetaSliceWidget*>()->attachScene(s);
}

void ThetaSliceView::addObject(QWidget *sender, QPointF p, float t)
{
    emit objectRequested(sender, p, t);
}

void ThetaSliceView::updateObject(int id, QPointF p, float t)
{
    emit objectChanged(id,p,t);
}
