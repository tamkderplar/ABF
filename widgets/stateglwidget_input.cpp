#include "stateglwidget.h"
#include <QApplication>
#include <QKeyEvent>
#define GLM_FORCE_RADIANS
#include "glm/gtc/matrix_transform.hpp"


void StateGLWidget::keyPressEvent(QKeyEvent *e)
{
    switch (e->key()) {
    case Qt::Key_Escape:
        QCoreApplication::instance()->quit();
        break;
    default:
        QOpenGLWidget::keyPressEvent(e);
        break;
    }
}

void StateGLWidget::mouseMoveEvent(QMouseEvent *e)
{
    glm::mat4 roty = glm::rotate(glm::mat4(),-(e->x()-cursorLastPos.x())/200.0f,glm::vec3(0.0f,1.0f,0.0f));
    glm::mat4 rotx = glm::rotate(glm::mat4(),-(e->y()-cursorLastPos.y())/200.0f,glm::vec3(1.0f,0.0f,0.0f));
    worldMat = rotx*roty*worldMat;
    cursorLastPos = e->pos();
    update();
}

void StateGLWidget::mousePressEvent(QMouseEvent *e)
{
    cursorLastPos = e->pos();
    update();
}
