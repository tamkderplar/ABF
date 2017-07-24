#include "scenewidget.h"

#include <QPainter>

#include "thetaslicewidget.h"
#include "contactwidget.h"

SceneWidget::SceneWidget(QWidget *parent) : QWidget(parent)
{
    marigin = 10.0;
}

void SceneWidget::attachScene(FreeSpaceBoundary *s)
{
    scene = s;
}

void SceneWidget::addObject(QWidget *sender, QPointF p, float t)
{
    objects.append({p,t,QColor(Qt::cyan)});
    objectOwners.append(sender);
    if(auto w=qobject_cast<ThetaSliceWidget*>(sender)){
        w->createObject(objects.size()-1,p);
    }
    if(auto w=qobject_cast<ContactWidget*>(sender)){
        w->createObject(objects.size()-1,p,t);
    }
    update();
}

void SceneWidget::updateObject(int id, QPointF p, float t)
{
    objects[id].pos=p;
    objects[id].theta=t;
    update();
}

void SceneWidget::paintEvent(QPaintEvent *)
{
    if(!scene)return;
    QPainter p(this);
    p.fillRect(rect(),Qt::black);
    p.setPen(QColor(Qt::white));
    QVector<QLineF> objE = scene->object().listEdges();
    QVector<QLineF> obsE = scene->obstacles().listEdges();

    QPointF objectCenter = scene->object().center();
    /*QPointF obstaclesCenter={0,0};
    for(int i=0;i<obsE.size();++i){
        obstaclesCenter+=obsE[i].pointAt(0.5);
    }
    obstaclesCenter/=obsE.size();*/

    p.drawLines(obsE);
    for(int i=0;i<objects.size();++i){
        p.setPen(objects[i].color);
        QTransform rot;
        rot.rotateRadians(objects[i].theta);
        for(int j=0;j<objE.size();++j){
            p.drawLine(rot.map(objE[j].translated(-objectCenter)).translated(+objects[i].pos));
        }
    }
}
