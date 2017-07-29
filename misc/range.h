#ifndef RANGE_H
#define RANGE_H

#include <experimental/optional>
namespace std{
    using namespace experimental;
}

struct Range{
    float begin;
    float end;
    std::optional<Range> intersect(const Range& other) const;
    bool contains(float) const;
};
//bool operator==(const Range& r,const Range& other);

#endif // RANGE_H
