#include "grapheditor.h"

#include <QPainter>
#include <QMouseEvent>
#include <QDebug>

GraphEditor::GraphEditor(QWidget *parent) : QWidget(parent)
{
    graph = nullptr;
    grabbed = -1;
    mouseMode = NullMode;
}

void GraphEditor::setGraph(Graph *g)
{
    graph = g;
}

void GraphEditor::mouseMoveEvent(QMouseEvent *e)
{
    if(grabbed==-1)return;
    if(mouseMode==NullMode)return;
    if(mouseMode==MovingPoint)graph->moveVertex(grabbed,e->pos());
    cursorLastPos = e->pos();
    emit updateOther();
    update();
}

void GraphEditor::mousePressEvent(QMouseEvent *e)
{
    if(e->button()==Qt::RightButton) mouseMode = MovingPoint;
    else if(e->button()==Qt::LeftButton) mouseMode = NewLine;
    else mouseMode = NullMode;
    grabbed = graph->findNearestIndex(e->pos());
    if(grabbed == -1)mouseMode = NullMode;
    cursorLastPos = e->pos();
    update();
}

void GraphEditor::mouseDoubleClickEvent(QMouseEvent *e)
{
    graph->addVertex(e->pos());
    cursorLastPos = e->pos();
    update();
}

void GraphEditor::mouseReleaseEvent(QMouseEvent *e)
{
    if(mouseMode==NewLine){
        int second = graph->findNearestIndex(e->pos());
        if(second!=-1){
            graph->toggleEdge(grabbed,second);
            emit updateOther();
        }
    }
    mouseMode = NullMode;
    cursorLastPos = e->pos();
    update();
}

void GraphEditor::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.fillRect(rect(),Qt::black);
    //p.setWindow(-width()/2,-height()/2,width(),height());
    //p.setViewport(-width()/2,-height()/2,width(),height());
    if(!graph){
        qWarning() << "no graph";
        return;
    }
    p.setPen(QColor(Qt::white));
    if(grabbed!=-1 && mouseMode==NewLine){
        p.drawLine(cursorLastPos,graph->listVertices()[grabbed]);
    }

    p.setPen(QColor(Qt::white));
    for(QPointF&pf:graph->listVertices()){
        p.drawEllipse(pf,5,5);
    }
    auto edges=graph->listEdges();
    for(QLineF&l:edges){
        p.drawLine(l);
    }
    //
    QPointF center = graph->center();
    p.setPen(QColor(Qt::blue));
    p.drawEllipse(center,4,4);
}
