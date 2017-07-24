#ifndef THETASLICEWIDGET_H
#define THETASLICEWIDGET_H

#include <QWidget>
#include <QMap>

#include "freespaceboundary.h"

class ThetaSliceWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ThetaSliceWidget(QWidget *parent = 0);

    void attachScene(FreeSpaceBoundary* s);

private:
    FreeSpaceBoundary* scene;
    float theta;
    QMap<int,QPointF> objectsPos;
    //
    int grabbed;
    //QPointF cursorLastPos;

signals:
    void objectRequested(QWidget*sender,QPointF,float);
    void objectChanged(int id,QPointF,float);

public slots:
    void createObject(int id,QPointF);
    void setAngle1024(int t);
    void setAngle(float t);

private:
    QVector<QLineF> genCorrectContacts() const;

protected:
    void mouseMoveEvent(QMouseEvent*);
    void mousePressEvent(QMouseEvent*);
    void mouseDoubleClickEvent(QMouseEvent*);
    void mouseReleaseEvent(QMouseEvent*);
    void paintEvent(QPaintEvent*);
};

#endif // THETASLICEWIDGET_H
