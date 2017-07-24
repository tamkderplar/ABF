#include "bfpface.h"

#include <QTransform>

BFPFace::BFPFace(QLineF objLine, QLineF obsLine):
    objectLine(objLine),
    obstacleLine(obsLine)
{

}

BFPFace::BFPFace(const BFPFace &other):
    objectLine(other.objectLine),
    obstacleLine(other.obstacleLine)
{

}

QPointF BFPFace::f00(float t)
{
    QTransform mat = QTransform().rotateRadians(t);
    return obstacleLine.p1()-objectLine.p1()*mat;
}

QPointF BFPFace::f01(float t)
{
    QTransform mat = QTransform().rotateRadians(t);
    return obstacleLine.p1()-objectLine.p2()*mat;
}

QPointF BFPFace::f10(float t)
{
    QTransform mat = QTransform().rotateRadians(t);
    return obstacleLine.p2()-objectLine.p1()*mat;
}

QPointF BFPFace::f11(float t)
{
    QTransform mat = QTransform().rotateRadians(t);
    return obstacleLine.p2()-objectLine.p2()*mat;
}
