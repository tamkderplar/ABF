#include "contactwidget.h"

#include <QPainter>
#include <QMouseEvent>
#include <experimental/optional>
#include "doublecontactfunction.h"
#include "restrictedregion.h"
#include "misc/misc.h"

#include <QDebug>


ContactWidget::ContactWidget(QWidget *parent) : QWidget(parent)
{
    contactType = Contact::NoContact;
}

void ContactWidget::attachScene(FreeSpaceBoundary *s)
{
    scene = s;
}

void ContactWidget::createObject(int id, QPointF p,float t)
{
    QPointF pos{t,p.y()};
    //TODO: fix second coordinate
    objectsPos.insert(id,fromPaintCoords().map(pos));
    update();
}

void ContactWidget::setContact(int edgeID, int vertexID,
                               Contact::ContactType type)
{
    contactEdgeID = edgeID;
    contactVertexID = vertexID;
    contactType = type;
    updateContact();
    update();
}

void ContactWidget::updateContact(){
    if(contactType!=Contact::NoContact) {
        QVector<QLineF> E;
        QVector<QPointF> V;
        if(contactType==Contact::VertexEdge) {
            V = scene->object().listVertices();
            E = scene->obstacles().listEdges();
        } else if(contactType==Contact::EdgeVertex) {
            E = scene->object().listEdges();
            V = scene->obstacles().listVertices();
        }
        if( contactEdgeID >= 0 && contactEdgeID < E.size() &&
            contactVertexID >= 0 && contactVertexID < V.size()) {
            contact = { E[contactEdgeID], V[contactVertexID], contactType };
            return;
        }
    }
    contact.type = Contact::NoContact;
}

void ContactWidget::mouseMoveEvent(QMouseEvent *e)
{
    if(grabbed==-1)return;
    objectsPos[grabbed] = fromPaintCoords().map(QPointF{e->pos()});
    
    float p = objectsPos[grabbed].y();
    float theta = objectsPos[grabbed].x();
    QPointF pos;
    if(contact.type==Contact::VertexEdge) {
        QTransform t;
        t.rotateRadians(theta);
        pos = contact.edge.pointAt(p)-t.map(contact.vertex-scene->object().center());
    } else if(contact.type==Contact::EdgeVertex) {
        QTransform t;
        t.rotateRadians(theta);
        pos = contact.vertex-t.map(contact.edge.pointAt(p)-scene->object().center());
    }

    emit objectChanged(grabbed,pos,theta);
    update();
}

void ContactWidget::mousePressEvent(QMouseEvent *e)
{
    int ret_id = -1;
    float dist = 10;
    auto map = toPaintCoords();
    for(int id:objectsPos.keys()){
        QPointF dir = map.map(objectsPos[id])-e->pos();
        float newdist2 = dir.x()*dir.x()+dir.y()*dir.y();
        if(newdist2<dist*dist){
            dist = std::sqrt(newdist2);
            ret_id = id;
        }
    }
    grabbed = ret_id;
}

void ContactWidget::mouseDoubleClickEvent(QMouseEvent *e)
{
    //TODO: removing, snapping to functions

    QPointF pos = fromPaintCoords().map(QPointF{e->pos()});
    float p = pos.y();
    float theta = pos.x();
    if(contact.type==Contact::VertexEdge) {
        QTransform t;
        t.rotateRadians(theta);
        pos = contact.edge.pointAt(p)-t.map(contact.vertex-scene->object().center());
    } else if(contact.type==Contact::EdgeVertex) {
        QTransform t;
        t.rotateRadians(theta);
        pos = contact.vertex-t.map(contact.edge.pointAt(p)-scene->object().center());
    }

    emit objectRequested(this,pos,theta);
}

void ContactWidget::mouseReleaseEvent(QMouseEvent *)
{

    if(grabbed==-1)return;
    float tp = 2.0*glm::pi<float>();
    float xpos = objectsPos[grabbed].x();
    objectsPos[grabbed].setX(std::fmod(std::fmod(xpos,tp)+tp,tp));
    grabbed = -1;
}

void ContactWidget::paintEvent(QPaintEvent *)
{
    if(!scene)return;
    updateContact();
    PainterEX paint(this);
    if(contact.type==Contact::NoContact){
        paint.error();
        return;
    }
    paint.color(QColor(Qt::white));

    QVector<QLineF> objE = scene->object().listEdges();
    QVector<QLineF> obsE = scene->obstacles().listEdges();
    QVector<QPointF> objV = scene->object().listVertices();
    QVector<QPointF> obsV = scene->obstacles().listVertices();

    QVector<Contact> contacts;
    for(int i=0;i<objE.size();++i)
        for(int s=0;s<obsV.size();++s)
        {
            contacts.append(Contact{objE[i],obsV[s],Contact::EdgeVertex});
        }
    for(int i=0;i<objV.size();++i)
        for(int s=0;s<obsE.size();++s)
        {
            contacts.append(Contact{obsE[s],objV[i],Contact::VertexEdge});
        }

    Contact c=contact;

    QList<RestrictedRegion> regions;
    for(const QLineF& e: objE){
        for(const QLineF& f: obsE){
            //skip if too far
            if(contactUnreachable(e,f))continue;
            //skip if any generated contact is same as original
            if(contactNotDifferentEnough(e,f))continue;
            //
            regions.append({c,e,f});
            paint.color(Qt::darkRed);
            //paint.region(regions.last());
            /*paint.color(Qt::darkYellow);
            paint.verticalLine(glm::pi<float>()+*rat1.undefinedValue());
            paint.color(Qt::yellow);
            paint.verticalLine(*rat2.undefinedValue());
            paint.color(QColor(Qt::white));*/
        }
    }
    for(const RestrictedRegion&rr:regions){
        paint.color(Qt::green);
        for(auto&s:rr.ceiling()){
            paint.plot(s.r,s.dcf);
        }
        paint.color(Qt::red);
        for(auto&s:rr.flooring()){
            paint.plot(s.r,s.dcf);
        }
        paint.color(Qt::yellow);
        for(auto&s:rr.ceiling()){
            paint.circle({s.r.begin,s.dcf(s.r.begin)});
            paint.circle({s.r.end,s.dcf(s.r.end)});
        }
        for(auto&s:rr.flooring()){
            paint.circle({s.r.begin,s.dcf(s.r.begin)});
            paint.circle({s.r.end,s.dcf(s.r.end)});
        }
    }

    paint.color(Qt::magenta);
    for(int i=0;i<regions.size();++i){
        for(int j=i+1;j<regions.size();++j){
            RestrictedRegion&rr1 = regions[i];
            RestrictedRegion&rr2 = regions[j];
            for(auto&s1:rr1.ceilingAndFlooring()){
                for(auto&s2:rr2.ceilingAndFlooring()){
                    for(float t:s1.intersect(s2)){
                        paint.circle({t,s1.dcf(t)});
                        paint.circle({t,s2.dcf(t)});
                        paint.verticalLine(t,{s1.dcf(t),s2.dcf(t)});
                    }
                }
            }
        }
    }
    for(int i=0;i<width();i+=10){
        paint.color(QColor(Qt::red));
        paint.plotstep(i,0,0);
        paint.color(QColor(Qt::green));
        paint.plotstep(i,1,1);
    }
    for(Contact d:contacts){
        if(d==c)continue;
        DoubleContactFunction dcf{c,d};
        for(float prlll : dcf.parallelMovementAngle())
        {
            paint.color(QColor(Qt::yellow));
            paint.verticalLine(prlll);
            paint.color(QColor(Qt::white));
        }
    }
    for(Contact d:contacts){
        if(d==c)continue;
        DoubleContactFunction dcf{c,d};
        if (std::optional<float> undef = dcf.undefinedValue())
        if (((c.type==d.type && c.edge==d.edge) ||
             (c.type!=d.type &&
              (c.vertex==d.edge.p1() || c.vertex==d.edge.p2())))){
            paint.color(QColor(Qt::magenta));
            paint.verticalLine((*undef));
            paint.verticalLine((*undef)+glm::pi<float>());
            paint.color(QColor(Qt::white));
        }
        paint.color(QColor(Qt::white));
    }
    paint.color(QColor(Qt::cyan));
    for(auto&point:objectsPos){
        paint.circle(point);
    }
}

bool ContactWidget::contactUnreachable(QLineF e, QLineF f) const
{
    //MAYBE: remove divisions and square roots to optimize?
    Contact c=contact;
    QLineF statedge = c.type==Contact::EdgeVertex?e:f;
    QLineF rotedge = c.type==Contact::VertexEdge?e:f;
    float s[2] = {  cross2({c.edge.p1(),statedge.p1()},c.edge)/c.edge.length(),
                    cross2({c.edge.p1(),statedge.p2()},c.edge)/c.edge.length()};
    float d[2] = { std::fabs(s[0]), std::fabs(s[1]) };
    float dmin = (s[0]*s[1]<0.0)?0.0:qMin(d[0],d[1]);
    float r[2] = {  QLineF{c.vertex,rotedge.p1()}.length(),
                    QLineF{c.vertex,rotedge.p2()}.length(),};
    float rmax = qMax(r[0],r[1]);
    if(s[0]*s[1]>0.0){
        if(dmin>rmax){
            //zero intervals, no possible solutions;
            return true;
        }
    }
    return false;
}

bool ContactWidget::contactNotDifferentEnough(QLineF e, QLineF f) const
{
    Contact c=contact;
    Contact ev1{e,f.p1(),Contact::EdgeVertex}, ev2{e,f.p2(),Contact::EdgeVertex};
    Contact ve1{f,e.p1(),Contact::VertexEdge}, ve2{f,e.p2(),Contact::VertexEdge};
    return (ev1==c || ev2==c || ve1==c || ve2==c);
}

QTransform ContactWidget::fromPaintCoords() const
{
    const int marigin = 10;
    return QTransform(glm::pi<double>()*2.0/width(),0,0,1.0/(height()-1-2*marigin),0,-marigin/float(height()-1-2*marigin));
}

QTransform ContactWidget::toPaintCoords() const
{
    const int marigin = 10;
    return QTransform(0.5*width()/glm::pi<double>(),0,0,(height()-1-2*marigin),0,marigin);
}

ContactWidget::PainterEX::PainterEX(ContactWidget *w)
    :p(w)
{
    map = w->toPaintCoords();
    p.fillRect(w->rect(),Qt::black);
}

void ContactWidget::PainterEX::verticalLine(float theta, Range r)
{
    theta = flipToRange(0,glm::two_pi<float>(),theta);
    p.drawLine(map.map(QPointF{theta,r.begin}),
               map.map(QPointF{theta,r.end}));
}

void ContactWidget::PainterEX::line(QPointF a, QPointF b)
{
    p.drawLine(map.map(a),
               map.map(b));
}

void ContactWidget::PainterEX::plot(float arg0, float arg1,
                                    const DoubleContactFunction& dcf)
{
    QPointF p0 = map.map(QPointF{arg0,dcf(arg0)});
    QPointF p1 = map.map(QPointF{arg1,dcf(arg1)});
    int start = std::ceil(p0.x());
    int end = std::floor(p1.x());
    if(start==end){
        p.drawLine(p0,map.map(QPointF{arg1,dcf(arg1)}));
        return;
    }
    float theta = start*2*glm::pi<float>()/p.window().width();
    for(int i=start; i<end; i++){
        p1 = map.map(QPointF{theta,dcf(theta)});
        p.drawLine(p0,p1);
        p0 = p1;
        theta = (i+1)*2*glm::pi<float>()/p.window().width();
    }
    p.drawLine(p0, map.map(QPointF{arg1,dcf(arg1)}));
}

void ContactWidget::PainterEX::plot(Range arg,
                                    const DoubleContactFunction& dcf)
{
    plot(arg.begin,arg.end,dcf);
}

void ContactWidget::PainterEX::plotstep(int arg0, float t0, float t1)
{
    t0 = map.map(QPointF{0,t0}).y();
    t1 = map.map(QPointF{0,t1}).y();
    p.drawLine(QPointF{double(arg0),t0},
               QPointF{double(arg0+1),t1});
}

void ContactWidget::PainterEX::circle(QPointF pos,int r)
{
    QPointF center = map.map(pos);
    int w = p.window().width();
    center.setX(flipToRange(0,w,center.x()));
    p.drawEllipse(center,r,r);
    if(center.x()<=r){
        center.setX(center.x()+w);
        p.drawEllipse(center,r,r);
    }
    if(center.x()>=w-r){
        center.setX(center.x()-w);
        p.drawEllipse(center,r,r);
    }
}

void ContactWidget::PainterEX::color(QColor c)
{
    p.setPen(c);
}

void ContactWidget::PainterEX::error()
{
    float twopi = glm::pi<float>()*2.f;
    p.setPen(QColor(Qt::red));
    p.drawLine(map.map(QPointF{0,0}),map.map(QPointF{twopi,1}));
    p.drawLine(map.map(QPointF{0,1}),map.map(QPointF{twopi,0}));
}
