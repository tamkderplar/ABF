#include "abfexception.h"

ABFException::ABFException(const char *w): w(w) { }

const char *ABFException::what() const noexcept
{
    return w;
}
