#ifndef GRAPHEDITOR_H
#define GRAPHEDITOR_H

#include <QWidget>

#include "graph.h"

class GraphEditor : public QWidget
{
    Q_OBJECT
public:
    explicit GraphEditor(QWidget *parent = 0);

    void setGraph(Graph*);
protected:
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void paintEvent(QPaintEvent *event);
private:
    Graph* graph;
    //
    int grabbed;
    QPointF cursorLastPos;
    enum Mode{
        NullMode,
        MovingPoint,
        NewLine
    } mouseMode;
signals:
    void updateOther();

public slots:
};

#endif // GRAPHEDITOR_H
