#ifndef GRAPH_H
#define GRAPH_H

#include <QVector>
#include <QPointF>
#include <QLineF>
#include <QRectF>

#include "glm.h"

class Graph
{
public:
    Graph();

    void addVertex(QPointF);
    void addEdge(int v0,int v1);
    void toggleEdge(int v0,int v1);
    QPointF findNearest(QPointF) const;
    int findNearestIndex(QPointF) const;
    void moveVertex(int ,QPointF);
    QVector<QPointF> listVertices() const;
    QVector<QLineF> listEdges() const;
    QVector<QPoint> listEdgesRaw() const;
    QPointF center() const;
    void clear();

private:
    const float error = 5.0f;
    QVector<QPointF> vertices;
    QVector<glm::int2> edges;
    QRectF boundingbox;
};

#endif // GRAPH_H
