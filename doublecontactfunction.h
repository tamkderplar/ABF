#ifndef DOUBLECONTACTFUNCTION_H
#define DOUBLECONTACTFUNCTION_H

#include "freespaceboundary.h"

#include <QDebug>

#include "misc/range.h"

struct FreeContactEdge{
    struct Vertex{
        float theta;
        int type;
    } begin, end;
    //std::optional<FreeContactEdge> intersect(const FreeContactEdge& other) const;
};

class DoubleContactFunction
{
    Contact::ContactType originalContactType;
    Contact::ContactType drawingContactType;
    QPointF J,K;
    QLineF H,G;
    float A,B,C,D,E;
    std::optional<glm::float2> borderValues[2];
    DoubleContactFunction();
public:
    DoubleContactFunction(Contact original, Contact drawing);
    //DoubleContactFunction(const DoubleContactFunction &);
    //DoubleContactFunction& operator=(const DoubleContactFunction &);

    bool isTrigLinear() const;
    bool isTrigRational() const;
    bool isValid() const;

    float operator()(float theta) const;
    float atZero() const;

    std::optional<float> undefinedValue() const;

    std::optional<glm::float2> zeroes() const;
    std::optional<glm::float2> ones() const;

    bool hasParallelMovement() const;
    QVector<float> parallelMovementAngle() const;
    //QVector<float> parallelMovement(float theta) const;

    QVector<Range> valued01() const;

    friend class FreeSpaceBoundary;
    friend class ContactWidget;
    friend class TestDCF;
private:
    void computeZeroes();
    void computeOnes();
    static std::optional<glm::float2> solveLinearTrig(float A, float B, float C, float base=0.0f);
};

#endif // DOUBLECONTACTFUNCTION_H
