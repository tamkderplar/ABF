#ifndef BFPFACE_H
#define BFPFACE_H

#include <QLineF>

class BFPFace
{
public:
    BFPFace(QLineF objLine,QLineF obsLine);
    BFPFace(const BFPFace&other);
    QPointF f00(float t);
    QPointF f01(float t);
    QPointF f10(float t);
    QPointF f11(float t);

private:
    QLineF objectLine, obstacleLine;
};

#endif // BFPFACE_H
