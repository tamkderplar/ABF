#ifndef CONTACTWIDGET_H
#define CONTACTWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QMap>
#include "freespaceboundary.h"
#include "misc/range.h"

class DoubleContactFunction;

class ContactWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ContactWidget(QWidget *parent = 0);

    void attachScene(FreeSpaceBoundary* s);

private:
    FreeSpaceBoundary* scene;
    int contactEdgeID;
    int contactVertexID;
    Contact::ContactType contactType;
    Contact contact;
    QMap<int,QPointF> objectsPos;
    int grabbed;
    //QVector<QVector<Range>> drawn;

signals:
    void objectRequested(QWidget*sender,QPointF,float);
    void objectChanged(int id,QPointF,float);

public slots:
    void createObject(int id,QPointF,float);
    //void setContact(FreeSpaceBoundary::Contact c);
    void setContact(int edgeID, int vertexID,
                    Contact::ContactType type);
private:
    void updateContact();

protected:
    void mouseMoveEvent(QMouseEvent*);
    void mousePressEvent(QMouseEvent*);
    void mouseDoubleClickEvent(QMouseEvent*);
    void mouseReleaseEvent(QMouseEvent*);
    void paintEvent(QPaintEvent*);

private:
    QTransform fromPaintCoords() const;
    QTransform toPaintCoords() const;
    class PainterEX{
        QPainter p;
        QTransform map;
    public:
        PainterEX(ContactWidget*);
        void verticalLine(float theta, Range r = {0.0,1.0});
        void line(QPointF a, QPointF b);
        void plot(float arg0, float arg1, const DoubleContactFunction&);
        void plot(Range arg, const DoubleContactFunction&);
        void plotstep(int arg0, float t0, float t1);
        void circle(QPointF p,int r=5);
        void color(QColor c);
        void error();
    };
};

#endif // CONTACTWIDGET_H
