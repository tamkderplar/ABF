#ifndef CONTACT_H
#define CONTACT_H


#include <QPointF>
#include <QLineF>


struct Contact
{
    QLineF edge;
    QPointF vertex;
    enum ContactType{
        NoContact,
        EdgeVertex,
        VertexEdge,
    } type;
    bool operator!=(const Contact& other);
    bool operator==(const Contact& other);
};

#endif // CONTACT_H
