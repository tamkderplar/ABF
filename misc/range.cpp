#include "range.h"

#include <QDebug>

std::optional<Range> Range::intersect(const Range &other) const {
    //qDebug() << "intersect:(" <<begin<<","<<end<<"),("<<
      //          other.begin<<","<<other.end<<")";
    if (end<=other.begin || other.end<=begin){
        //qDebug() << "result: none";
        return std::nullopt;
    }
    Range r = {qMax(begin,other.begin),qMin(end,other.end)};
    //qDebug() << "result: ("<<r.begin<<","<<r.end<<")";
    return {r};
}

bool Range::contains(float t) const
{
    return begin<=t && t<=end;
}

/*bool operator==(const Range &r, const Range &other){
    return r.begin == other.begin && r.end == other.end;
}*/
