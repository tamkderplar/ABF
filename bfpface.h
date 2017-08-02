#ifndef BFPFACE_H
#define BFPFACE_H

#include "doublecontactfunction.h"

class BFPFace
{
public:
    BFPFace(Range r,DoubleContactFunction t,DoubleContactFunction b);
    BFPFace(const BFPFace&other);
    Range range() const;
    DoubleContactFunction top() const;
    DoubleContactFunction bottom() const;

private:
    Range r;
    DoubleContactFunction t, b;
};

#endif // BFPFACE_H
