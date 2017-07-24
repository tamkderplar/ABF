#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>

#include "glm.h"
#include "freespaceboundary.h"

class StateGLWidget:public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT
public:
    explicit StateGLWidget(QWidget*parent=0);

    void attachScene(FreeSpaceBoundary* s);

protected:
    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void paintGL();

    void keyPressEvent(QKeyEvent* e);
    void mouseMoveEvent(QMouseEvent *e);
    void mousePressEvent(QMouseEvent *e);

private:
    void drawFull(glm::mat4 fullMat, int drawSize);
    void initShaderPrograms();
    bool initShaderProgram(QOpenGLShaderProgram&shader,
                           QMap<QOpenGLShader::ShaderTypeBit,QString> sourceNames);
    bool initBuffers();

    QPoint cursorLastPos;
    glm::mat4 worldMat, projMat, viewportMat;

    QOpenGLShaderProgram shader_ev;
    QOpenGLShaderProgram shader_ve;
    QOpenGLShaderProgram shader_wires;
    //WQOpenGLShaderProgram shader_normals;
    QOpenGLBuffer vertexBuffer;
    QOpenGLBuffer indexBuffer;

    FreeSpaceBoundary* scene;
};

#endif // GLWIDGET_H
