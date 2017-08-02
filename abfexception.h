#ifndef ABFEXCEPTION_H
#define ABFEXCEPTION_H

#include <exception>

class ABFException : public std::exception
{
    const char *w;
public:
    ABFException(const char*);
    const char *what() const noexcept;
};

#endif // ABFEXCEPTION_H
