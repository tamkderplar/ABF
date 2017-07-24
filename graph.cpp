#include "graph.h"

Graph::Graph()
{

}

void Graph::addVertex(QPointF p)
{
    if (vertices.empty()){
        boundingbox = {p,p};
    } else {
        boundingbox = boundingbox.united({p,vertices[0]}).normalized();
    }
    vertices.append(p);
}

void Graph::addEdge(int v0, int v1)
{
    if(v0==v1)return;
    if(v0<0||v0>=vertices.size())return;
    if(v1<0||v1>=vertices.size())return;
    if(edges.contains(glm::int2{v0,v1}))return;
    if(edges.contains(glm::int2{v1,v0}))return;
    edges.append(glm::int2{v0,v1});
}

void Graph::toggleEdge(int v0, int v1)
{
    if(v0==v1)return;
    if(v0<0||v0>=vertices.size())return;
    if(v1<0||v1>=vertices.size())return;
    if(edges.contains(glm::int2{v0,v1})){
        edges.removeOne(glm::int2{v0,v1});
        return;
    }
    if(edges.contains(glm::int2{v1,v0})){
        edges.removeOne(glm::int2{v1,v0});
        return;
    }
    edges.append(glm::int2{v0,v1});
}

QPointF Graph::findNearest(QPointF p) const
{
    return vertices[findNearestIndex(p)];
}

int Graph::findNearestIndex(QPointF p) const
{
    int id = -1;
    float dist=10;
    for(int i=0;i<vertices.size();++i){
        QPointF dir = vertices[i]-p;
        float newdist2 = dir.x()*dir.x()+dir.y()*dir.y();
        if(newdist2<dist*dist){
            dist = std::sqrt(newdist2);
            id = i;
        }
    }
    return id;
}

void Graph::moveVertex(int v, QPointF p)
{
    if(v<0||v>=vertices.size())return;
    vertices[v] = p;
    //TODO: store in a tree for faster AABB update
    boundingbox = {vertices[0],vertices[0]};
    for(QPointF r:vertices){
        boundingbox = boundingbox.united({r,vertices[0]}).normalized();
    }
}

QVector<QPointF> Graph::listVertices() const
{
    return vertices;
}

//ENDGAME: look at usage, generating list is O(e)
QVector<QLineF> Graph::listEdges() const
{
    QVector<QLineF> ret;
    std::for_each(edges.begin(),edges.end(),
                [&ret,this](glm::int2 ii){
                    ret.append(QLineF{vertices[ii.x],
                                      vertices[ii.y]});
                });
    return ret;
}

QVector<QPoint> Graph::listEdgesRaw() const
{
    QVector<QPoint> ret;
    std::for_each(edges.begin(),edges.end(),
                [&ret,this](glm::int2 ii){
                    ret.append(QPoint{ii.x,ii.y});
                });
    return ret;
}

QPointF Graph::center() const
{
    return boundingbox.center();
}

void Graph::clear()
{
    edges.clear();
    vertices.clear();
    boundingbox = QRectF();
}
