#include "contact.h"

bool Contact::operator!=(const Contact &other)
{
    return type!=other.type || vertex!=other.vertex || edge!=other.edge;
}

bool Contact::operator==(const Contact &other)
{
    return !(*this!=other);
}
