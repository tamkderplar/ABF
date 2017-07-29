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
        QVector<float> intersect(const Segment&s);
    };
    QVector<Segment> ceiling() const;
    QVector<Segment> flooring() const;
    QVector<Segment> ceilingAndFlooring() const;
private:
    QVector<Segment> top;
    QVector<Segment> bottom;
    static QMap<int,int> maxiter;
public:
    static QVector<float> solveQuadraticTrig(double x2,double y2,double xy,double x,double y,double u);
    static std::optional<glm::float2> solveLinearTrig(float A, float B, float C, float base=0.0f);
    static QVector<float> solveQuartic(double a,double b,double c,double d,double e);
};

#endif // RESTRICTEDREGION_H
