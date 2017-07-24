#include "thetaslicewidget.h"

#include <QPainter>
#include <QMouseEvent>
#include "cppitertools/itertools.hpp"
#include "misc/misc.h"

ThetaSliceWidget::ThetaSliceWidget(QWidget *parent) : QWidget(parent)
{
    scene = nullptr;
    theta = 0;
    grabbed = -1;
}

void ThetaSliceWidget::attachScene(FreeSpaceBoundary *s)
{
    scene = s;
}

void ThetaSliceWidget::createObject(int id,QPointF p)
{
    QPointF obstaclesCenter = scene->obstacles().center();
    objectsPos.insert(id,p-obstaclesCenter);
    update();
}

void ThetaSliceWidget::setAngle1024(int t)
{
    theta = (2*t)*glm::pi<float>()/1024.0;
    QPointF obstaclesCenter = scene->obstacles().center();
    for(int id:objectsPos.keys()){
        emit objectChanged(id,objectsPos[id]+obstaclesCenter,theta);
    }
    update();
}

void ThetaSliceWidget::setAngle(float t)
{
    theta = t;
    QPointF obstaclesCenter = scene->obstacles().center();
    for(int id:objectsPos.keys()){
        emit objectChanged(id,objectsPos[id]+obstaclesCenter,t);
    }
    update();
}

/*QVector<QLineF> ThetaSliceWidget::genCorrectContacts() const
{

}*/

void ThetaSliceWidget::mouseMoveEvent(QMouseEvent *e)
{
    if(grabbed==-1)return;
    //
    QPointF obstaclesCenter = scene->obstacles().center();
    //
    QPointF mid = QPointF{width()/2.0,height()/2.0};
    objectsPos[grabbed] = e->pos()-mid;
    emit objectChanged(grabbed,e->pos()-mid+obstaclesCenter,theta);
    update();
}

void ThetaSliceWidget::mousePressEvent(QMouseEvent *e)
{
    int ret_id = -1;
    float dist = 10;
    QPointF mid = QPointF{width()/2.0,height()/2.0};
    for(int id:objectsPos.keys()){
        QPointF dir = objectsPos[id]-e->pos()+mid;
        float newdist2 = dir.x()*dir.x()+dir.y()*dir.y();
        if(newdist2<dist*dist){
            dist = std::sqrt(newdist2);
            ret_id = id;
        }
    }
    grabbed = ret_id;
}

void ThetaSliceWidget::mouseDoubleClickEvent(QMouseEvent *e)
{
    QPointF obstaclesCenter = scene->obstacles().center();
    //TODO: removing
    QPointF mid = QPointF{width()/2.0,height()/2.0};
    emit objectRequested(this,e->pos()-mid+obstaclesCenter,theta);
}

void ThetaSliceWidget::mouseReleaseEvent(QMouseEvent *)
{
    grabbed = -1;
}

void ThetaSliceWidget::paintEvent(QPaintEvent *)
{
    if(!scene)return;
    QPainter p(this);
    p.fillRect(rect(),Qt::black);
    p.setWindow(-width(),-height(),width()*2,height()*2);
    p.setWindow(-width()/2,-height()/2,width(),height());
    //p.setViewport(-width()/2,-height()/2,width(),height());
    p.setPen(QColor(Qt::blue));
    p.drawEllipse(-5,-5,10,10);

    QVector<QLineF> object = scene->object().listEdges();
    QVector<QLineF> obstacles = scene->obstacles().listEdges();

    QPointF objectCenter = scene->object().center();
    for(int i=0;i<object.size();++i){
        object[i].translate(-objectCenter);
    }
    QPointF obstaclesCenter=scene->obstacles().center();
    for(int i=0;i<obstacles.size();++i){
        obstacles[i].translate(-obstaclesCenter);
    }

    QTransform t;
    t.rotateRadians(theta);
    for(QLineF&l:object){
        l.setP1(t.map(l.p1()));
        l.setP2(t.map(l.p2()));
    }

    QVector<QLineF> contactsVE;
    QVector<QLineF> contactsEV;
    {
        QSet<QPointF> objV_filtered;
        for(QLineF edge : object){
            objV_filtered.insert(edge.p1());
            objV_filtered.insert(edge.p2());
        }
        for(QPointF v : objV_filtered){
            for(QLineF edge : obstacles){
                contactsVE.append({edge.p1()-v,edge.p2()-v});
            }
        }
        QSet<QPointF> obsV_filtered;
        for(QLineF edge : obstacles){
            obsV_filtered.insert(edge.p1());
            obsV_filtered.insert(edge.p2());
        }
        for(QPointF v : obsV_filtered){
            for(QLineF edge : object){
                contactsEV.append({v-edge.p1(),v-edge.p2()});
            }
        }
    }

    p.setPen(QColor(Qt::white));

    //fill parallelograms
    for(int i=0;i<object.size();++i){
        for(int j=0;j<obstacles.size();++j){
            QPainterPath path(obstacles[j].p1()-object[i].p1());
            path.lineTo(obstacles[j].p2()-object[i].p1());
            path.lineTo(obstacles[j].p2()-object[i].p2());
            path.lineTo(obstacles[j].p1()-object[i].p2());
            path.closeSubpath();
            p.fillPath(path,Qt::red);
        }
    }

    //draw parallelogram boundaries
    //(i.e. cointacts, but sometimes more than once)
    /*for(int i=0;i<object.size();++i){
        for(int j=0;j<obstacles.size();++j){
            QPainterPath path(obstacles[j].p1()-object[i].p1());
            path.lineTo(obstacles[j].p2()-object[i].p1());
            path.lineTo(obstacles[j].p2()-object[i].p2());
            path.lineTo(obstacles[j].p1()-object[i].p2());
            path.closeSubpath();
            p.drawPath(path);
        }
    }*/
    //draw contacts(if draw_all)
    //p.drawLines(contactsVE);
    //p.drawLines(contactsEV);

    QVector<QLineF> contacts_parts;
    {
        auto cross2= [](QPointF a,QPointF b){
            return a.x()*b.y()-a.y()*b.x();
        };
        auto list_QLineF=[](QLineF edge){
            return QVector<QPointF>{edge.p1(),edge.p2()};
        };
        auto side_of=[cross2](QPointF p,QLineF edge){
            return cross2(p-edge.p1(),edge.p2()-edge.p1());
        };
        auto dir=[cross2](QLineF edge){
            return edge.p2()-edge.p1();
        };
        struct parallelogram{
            QPointF v;
            QPointF d1, d2;
        };
        auto inside=[side_of](QPointF point,parallelogram p){
            return qMax(
                side_of(point,{p.v,p.v+p.d1})*side_of(point,{p.v+p.d2,p.v+p.d1+p.d2}),
                side_of(point,{p.v,p.v+p.d2})*side_of(point,{p.v+p.d1,p.v+p.d1+p.d2})
                       );
        };

        for(QLineF contact : contactsVE){
            QVector<QPointF> inaccessible_ranges;
            for(QLineF edgeE : obstacles){
                //== is ok, bc we only need to remove explicitly parallel
                //but refactoring would make it better
                if(edgeE.dx()*contact.dy()==edgeE.dy()*contact.dx())continue;
                for(QLineF edgeI : object){
                    //parallelograms are pairs (edgeI,edgeE)
                    //  or rather Minkowski difference edgeE-edgeI
                    parallelogram p = {edgeE.p1()-edgeI.p1(),dir(edgeI),dir(edgeE)};
                    QVector<QPointF> corners;
                    for(auto&& pair:iter::product(list_QLineF(edgeE),
                                                  list_QLineF(edgeI))){
                        corners.append(std::get<0>(pair)-std::get<1>(pair));
                    }
                    QVector<QLineF> sides = {{corners[0],corners[1]}
                                            ,{corners[2],corners[3]}
                                            ,{corners[0],corners[2]}
                                            ,{corners[1],corners[3]}};
                    //skip if contact is inside parallelogram
                    //TODO:skip whole contact, it's invalid
                    //refactoring will make it easier
                    if (inside(contact.p1(),p)<0.0f &&
                        inside(contact.p2(),p)<0.0f){
                        inaccessible_ranges.append({0.0,1.0});
                        break;
                    }
                    //skip if parallelogram and contact don't intersect
                    bool contact_intersection = false;
                    QVector<glm::vec2> intersection_parameters;
                    QVector<float> contact_parameters;
                    for(auto side : sides){
                        float side_param,cont_param;
                        switch(contact.intersect(side,nullptr)){
                        case QLineF::NoIntersection:
                            side_param=cont_param=INFINITY;
                            break;
                        case QLineF::BoundedIntersection:
                            contact_intersection = true;
                            //fallthrough to paramater computation
                        case QLineF::UnboundedIntersection:
                            float dircross = cross2(dir(contact),dir(side));
                            QPointF startdir = side.p1()-contact.p1();
                            side_param = cross2(startdir,dir(contact))/dircross;
                            cont_param = cross2(startdir,dir(side))/dircross;
                        }
                        intersection_parameters.append({side_param,cont_param});
                        if(side_param<=1.0f && side_param>=0.0f)
                            contact_parameters.append(cont_param);
                    }
                    if(!contact_intersection)continue;
                    //compute and add inaccessible range
                    float *t0, *t1;
                    std::tie(t0, t1) = std::minmax_element(contact_parameters.begin()
                                                           ,contact_parameters.end());
                    inaccessible_ranges.append({*t0,*t1});
                }
            }
            std::sort(inaccessible_ranges.begin(),inaccessible_ranges.end(),
                  [](QPointF a,QPointF b){ return a.x()<b.x(); });
            float last_end = 0.0f;
            for(QPointF p : inaccessible_ranges){
                if(last_end < p.x()){
                    contacts_parts.append({contact.pointAt(last_end),
                                           contact.pointAt(p.x())});
                }
                if(last_end < p.y()){
                    last_end = p.y();
                }
                if(last_end > 1.0)break;
            }
            if(last_end<1.0){
                contacts_parts.append({contact.pointAt(last_end),
                                       contact.pointAt(1.0)});
            }
        }
        p.drawLines(contacts_parts);
        for(QLineF contact : contactsEV){
            for(QLineF edgeI : object){
                //== is ok, bc we only need to remove explicitly parallel
                //but refactoring would make it better
                if(edgeI.dx()*contact.dy()==edgeI.dy()*contact.dx())continue;
                for(QLineF edgeE : obstacles){
                    //parallelograms are pairs (edgeI,edgeE)
                    //  or rather Minkowski difference edgeE-edgeI
                }
            }
        }
    }

    //mark vertex-edge contacts
    /*p.setPen(QColor(Qt::yellow));
    for(int i=0;i<object.size();++i){
        for(int j=0;j<obstacles.size();++j){
            p.drawLine(obstacles[j].p1()-object[i].p1(),
                       obstacles[j].p2()-object[i].p1());
            p.drawLine(obstacles[j].p2()-object[i].p2(),
                       obstacles[j].p1()-object[i].p2());
        }
    }*/

    //draw object representants
    p.setPen(QColor(Qt::cyan));
    for(auto&point:objectsPos){
        //p.drawEllipse(point-QPointF{width()/2.0,height()/2.0},5,5);
        p.drawEllipse(point,5,5);
    }
}
