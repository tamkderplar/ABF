#ifndef MISC_H
#define MISC_H

#include <QPointF>
#include <QDebug>

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
    Q_ASSERT(b>a);
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
