#include "bfpface.h"

#include <QTransform>

BFPFace::BFPFace(Range r,DoubleContactFunction t,DoubleContactFunction b):
    r(r),
    t(t),
    b(b)
{

}

BFPFace::BFPFace(const BFPFace &other):
    r(other.r),
    t(other.t),
    b(other.b)
{

}

Range BFPFace::range() const
{
    return r;
}

DoubleContactFunction BFPFace::top() const
{
    return t;
}

DoubleContactFunction BFPFace::bottom() const
{
    return b;
}
