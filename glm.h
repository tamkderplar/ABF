#ifndef GLM_H
#define GLM_H

#include "glm/glm.hpp"
#include "glm/gtx/compatibility.hpp"
typedef unsigned int uint;


inline uint qHash(glm::int2 val)
{
    return uint(val.x ^ (val.y<<16) ^ (val.y>>16));
}

inline uint qHash(glm::int2 val, uint seed)
{
    return uint(val.x ^ (val.y<<16) ^ (val.y>>16) ^ seed);
}

#endif // GLM_H
