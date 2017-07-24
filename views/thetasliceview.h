#ifndef THETASLICEVIEW_H
#define THETASLICEVIEW_H

#include <QWidget>

#include "freespaceboundary.h"

namespace Ui {
class ThetaSliceView;
}

class ThetaSliceView : public QWidget
{
    Q_OBJECT

public:
    explicit ThetaSliceView(QWidget *parent = 0);
    ~ThetaSliceView();

    void attachScene(FreeSpaceBoundary* s);

signals:
    //signals passed from slice widget to scene wigdet
    void objectRequested(QWidget*sender,QPointF,float);
    void objectChanged(int id,QPointF,float);

public slots:
    //slots passing signal to scene widget
    void addObject(QWidget*sender,QPointF,float);
    void updateObject(int id,QPointF,float);

private:
    Ui::ThetaSliceView *ui;
};

#endif // THETASLICEVIEW_H
