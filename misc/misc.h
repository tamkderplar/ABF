#ifndef MISC_H
#define MISC_H

#include <QPointF>
#include <QDebug>
#include "glm.h"
#include "range.h"
#include "abfexception.h"

#define ABF_ASSERT(X) abfAssert(X)

inline void abfAssert(bool x){
    if(!x){
        qDebug()<<x;
        throw ABFException("abfError");
    }
}

#define ABF_CHECK(X) abfCheck(X)
inline void abfCheck(bool x){
    if(!x){
        qDebug()<<x;
    }else{
        qDebug()<<x;
    }
}

inline QDebug operator<<(QDebug dbg, const glm::float2& f2){
    dbg.nospace() << "(" << f2.x << "," << f2.y << ")";
    return dbg.maybeSpace();
}

inline QDebug operator<<(QDebug dbg, const glm::float3& f3){
    dbg.nospace() << "(" << f3.x << "," << f3.y << "," << f3.z << ")";
    return dbg.maybeSpace();
}

template <typename T>
inline QDebug operator<<(QDebug dbg, const std::optional<T>& opt){
    if(!opt){
        dbg.nospace() << "<no value>";
    }else{
        dbg.nospace() << "<(" << *opt << ")>";
    }
    return dbg.maybeSpace();
}

inline uint qHash(QPointF a)
{
    const quint64& x = reinterpret_cast<const quint64&>(a.rx());
    const quint64& y = reinterpret_cast<const quint64&>(a.ry());
    quint64 hash = x ^ (y<<32) ^ (y>>32);
    return (hash>>32)^hash;
}

inline uint qHash(QPointF a, uint seed)
{
    const quint64& x = reinterpret_cast<const quint64&>(a.rx());
    const quint64& y = reinterpret_cast<const quint64&>(a.ry());
    quint64 hash = x ^ (y<<32) ^ (y>>32);
    return (hash>>32)^hash ^ seed;
}


inline float cross2(QPointF a,QPointF b){return a.x()*b.y()-a.y()*b.x();}
inline float cross2(QLineF a,QLineF b){return cross2(a.p2()-a.p1(),b.p2()-b.p1());}
//TODO: check if cross2 for QLineF is needed
inline float dot2(QPointF a,QPointF b){return a.x()*b.x()+a.y()*b.y();}
inline float dot2(QLineF a,QLineF b){return dot2(a.p2()-a.p1(),b.p2()-b.p1());}
//TODO: check if dot2 for QLineF is needed


inline float flipToRange(float a,float b, float t){
    ABF_ASSERT(b>a);
    //qDebug() << "bfflip" << "a:" << a << "b:"<<b<<"t:"<<t;
    /*while(t <  a){t+=b-a;}
    while(t >= b){t-=b-a;}
    //qDebug() << "fliped" << "a:" << a << "b:"<<b<<"t:"<<t;*/
    if(glm::isnan(t)){
        qDebug()<<"flipping impossible: t is nan";
    }
    t = std::fmod(std::fmod(t-a,b-a)+(b-a),b-a)+a;
    if(t==b)t=a;
    if(! (a<=t && t<b) ){
        throw;
    }
    return t;
}

#endif // MISC_H
