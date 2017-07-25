#ifndef RESTRICTEDREGION_H
#define RESTRICTEDREGION_H

#include "doublecontactfunction.h"

class RestrictedRegion
{
public:
    RestrictedRegion(Contact c, QLineF obj, QLineF obs);

    //QVector<BFPFace> subregions();
    struct Segment{
        Range r;
        DoubleContactFunction dcf;
    };
    QVector<Segment> ceiling() const;
    QVector<Segment> flooring() const;
private:

    QVector<Segment> top;
    QVector<Segment> bottom;
};

#endif // RESTRICTEDREGION_H
