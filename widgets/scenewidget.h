#ifndef SCENEWIDGET_H
#define SCENEWIDGET_H

#include <QWidget>

#include "freespaceboundary.h"

class SceneWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SceneWidget(QWidget *parent = 0);

    void attachScene(FreeSpaceBoundary* s);

    struct SceneObject{
        QPointF pos;
        float theta;
        QColor color;
    };

signals:

public slots:
    void addObject(QWidget*sender,QPointF,float);
    void updateObject(int id,QPointF,float);

protected:
    void paintEvent(QPaintEvent *);

private:
    FreeSpaceBoundary* scene;
    float marigin;
    QVector<SceneObject> objects;
    QVector<QWidget*> objectOwners;
};

#endif // SCENEWIDGET_H
