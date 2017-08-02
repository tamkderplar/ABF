#ifndef FREESPACEBOUNDARY_H
#define FREESPACEBOUNDARY_H

#include <QVector>

#include "bfpface.h"
#include "graph.h"
#include "glm.h"
#include "contact.h"

struct DoubleContactEdge{
    float theta1,theta2;
    Contact *c1,*c2;
    enum EdgeType{
        NO_TYPE = 1<<6,
        ZERO = 0,
        ONE = 1,
    }type;
};

class FreeSpaceBoundary
{
public:
    FreeSpaceBoundary();

    Graph& object();
    const Graph& object() const;
    Graph& obstacles();
    const Graph& obstacles() const;
    QVector<BFPFace> faces() const;
    void computeFaces();
private:
    Graph objectGraph;
    Graph obstaclesGraph;
    QVector<BFPFace> faceCache;


public:
    struct TripleContactVertex{
        float theta;
        Contact *c1,*c2,*c3;
        DoubleContactEdge *e1,*e2,*e3;
        enum VertexType{
            NO_TYPE = 1<<6,
            ZERO_ZERO = 0,
            ZERO_ONE = 1,
            ONE_ZERO = 2,
            ONE_ONE = 3,
        };
    };
};

QDataStream &operator<<(QDataStream &out, const FreeSpaceBoundary &fsb);
QDataStream &operator>>(QDataStream &in, FreeSpaceBoundary &fsb);

#endif // FREESPACEBOUNDARY_H
