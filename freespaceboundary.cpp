#include "freespaceboundary.h"

#include "doublecontactfunction.h"
#include "misc/misc.h"

#include <QDataStream>
#include <algorithm>
#include <QFile>

FreeSpaceBoundary::FreeSpaceBoundary()
{

}

Graph &FreeSpaceBoundary::object()
{
    return objectGraph;
}

const Graph &FreeSpaceBoundary::object() const
{
    return objectGraph;
}

Graph &FreeSpaceBoundary::obstacles()
{
    return obstaclesGraph;
}

const Graph &FreeSpaceBoundary::obstacles() const
{
    return obstaclesGraph;
}

QVector<BFPFace> FreeSpaceBoundary::faces() const
{
    QVector<BFPFace> ret;
    return ret;
}

template<typename Element>
class Vector2{
    QVector<Element> data;
    glm::int2 size;
public:
    Vector2(glm::int2 startsize){
        data.resize(startsize.x*startsize.y);
        size = startsize;
    }
    Element& operator[](glm::int2 id){
        return data[id.x*size.y+id.y];
    }

    /*class Proxy{
        int offset;
    public:
        Element& operator[](int y){
            return data[offset+y];
        }
    };
    const Proxy& operator[](int x){
        return Proxy(x*size.y);
    }*/
};

void FreeSpaceBoundary::computeFaces()
{
    struct BFPFace{
        float theta1,theta2;
        DoubleContactEdge *top, *bottom;
    };

    QVector<QLineF> objE = objectGraph.listEdges();
    QVector<QLineF> obsE = obstaclesGraph.listEdges();
    QVector<QPointF> objV = objectGraph.listVertices();
    QVector<QPointF> obsV = obstaclesGraph.listVertices();
    QVector<Contact> contacts;
    for(int i=0;i<objE.size();++i)
        for(int s=0;s<obsV.size();++s)
        {
            contacts.append(Contact{objE[i],obsV[s],
                                    Contact::EdgeVertex});
        }
    for(int i=0;i<objV.size();++i)
        for(int s=0;s<obsE.size();++s)
        {
            contacts.append(Contact{obsE[s],objV[i],
                                    Contact::VertexEdge});
        }
    QList<DoubleContactFunction> dcfs;
    //QMap<glm::int2,DoubleContactFunction> dcfs;
    for(Contact c : contacts){
        for(Contact d : contacts){
            if(c!=d){
                dcfs.append({c,d});
            }
        }
    }



    //construct unavailable faces
    //based on parallelograms
    QVector<BFPFace> restricted;
    for(Contact c:contacts){
        for(QLineF&e:objE)
            for(QLineF&f:obsE){
                //TODO: don't create new contacts, refer to dcfs or contacts
                Contact ev1{e,f.p1(),Contact::EdgeVertex}, ev2{e,f.p2(),Contact::EdgeVertex};
                Contact ve1{f,e.p1(),Contact::VertexEdge}, ve2{f,e.p2(),Contact::VertexEdge};
                if(ev1==c || ev2==c || ve1==c || ve2==c)continue;
                DoubleContactFunction dcfs[4] = {{c,ev1},{c,ev2},{c,ve1},{c,ve2}};
                DoubleContactFunction invs[4] = {{ev1,c},{ev2,c},{ve1,c},{ve2,c}};

                QLineF statedge = c.type==Contact::EdgeVertex?e:f;
                float s[2] = {  cross2({c.edge.p1(),statedge.p1()},c.edge)/c.edge.length(),
                                cross2({c.edge.p1(),statedge.p2()},c.edge)/c.edge.length()};
            }
    }
}

void FreeSpaceBoundary::saveAsError(const char *s) const
{
    QString filename(s);
    int n = 0;
    while(QFile(filename+".abf").exists()){
        filename = QString(s)+QString::number(n);
        ++n;
    }
    QFile file(filename+".abf");
    file.open(QIODevice::WriteOnly);
    QDataStream out(&file);
    out<<*this;
    file.close();
}

QDataStream &operator<<(QDataStream &out, const FreeSpaceBoundary &fsb)
{
    out << fsb.object().listVertices() << fsb.object().listEdgesRaw();
    out << fsb.obstacles().listVertices() << fsb.obstacles().listEdgesRaw();
    return out;
}

QDataStream &operator>>(QDataStream &in, FreeSpaceBoundary &fsb)
{
    fsb.object().clear();
    fsb.obstacles().clear();
    QVector<QPointF> verts;
    QVector<QPoint> edgeIDs;
    in >> verts >> edgeIDs;
    for(QPointF v : verts){
        fsb.object().addVertex(v);
    }
    for(QPoint e : edgeIDs){
        fsb.object().addEdge(e.x(),e.y());
    }
    in >> verts >> edgeIDs;
    for(QPointF v : verts){
        fsb.obstacles().addVertex(v);
    }
    for(QPoint e : edgeIDs){
        fsb.obstacles().addEdge(e.x(),e.y());
    }
    return in;
}
