#include "stateglwidget.h"

StateGLWidget::StateGLWidget(QWidget *parent) :
    QOpenGLWidget(parent),
    vertexBuffer(QOpenGLBuffer::VertexBuffer),
    indexBuffer(QOpenGLBuffer::IndexBuffer),
    scene(nullptr)
{
    worldMat = glm::mat4(1.0f);
    worldMat[1][1] = worldMat[2][2] = -1;
    projMat = glm::mat4(1.0f);
    projMat[2][3] = 1.0f/5.0f;
    viewportMat = glm::mat4(1.0f);
}

void StateGLWidget::attachScene(FreeSpaceBoundary *s)
{
    scene = s;
}

void StateGLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    QSurfaceFormat glFormat = QOpenGLWidget::format();
    if(glFormat.samples()==1)
        qWarning() << "Could not enable sample buffers";
    glClearColor(0.0f,0.0f,0.0f,1.0f);

    glEnable(GL_DEPTH_TEST);

    {
        uint vao;
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);
        //QOpenGLVertexArrayObject vao;
        //vao.create();
        //vao.bind();
    }

    initShaderPrograms();
    if(!initBuffers())return;
}

void StateGLWidget::resizeGL(int w, int h)
{
    float maxSize = qMax(w,h);
    viewportMat[0][0] = h/maxSize;
    viewportMat[1][1] = w/maxSize;
}

void StateGLWidget::paintGL()
{
    if(!vertexBuffer.bind()){
        qWarning() << "Could not bind vBuffer.";
        return;
    }
    if(!indexBuffer.bind()){
        qWarning() << "Could not bind iBuffer.";
        return;
    }
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    if(!scene)return;
    QVector<QLineF> object = scene->object().listEdges();
    QVector<QLineF> obstacles = scene->obstacles().listEdges();

    QPointF objectCenter={0,0};
    for(int i=0;i<object.size();++i){
        objectCenter+=object[i].pointAt(0.5);
    }
    objectCenter/=object.size();
    for(int i=0;i<object.size();++i){
        object[i].translate(-objectCenter);
    }
    QPointF obstaclesCenter={0,0};
    for(int i=0;i<obstacles.size();++i){
        obstaclesCenter+=obstacles[i].pointAt(0.5);
    }
    obstaclesCenter/=obstacles.size();
    for(int i=0;i<obstacles.size();++i){
        obstacles[i].translate(-obstaclesCenter);
    }

    //fill data
    QVector<float> data = QVector<float>(4*(object.size()+obstacles.size()));
    vertexBuffer.allocate(data.size()*sizeof(float));
    for(int i=0;i<object.size();++i){
        data[4*i+0] = object[i].p1().x()/width();
        data[4*i+1] = object[i].p1().y()/width();
        data[4*i+2] = object[i].p2().x()/width();
        data[4*i+3] = object[i].p2().y()/width();
    }
    int offset = 4*object.size();
    for(int j=0;j<obstacles.size();++j){
        data[offset+4*j+0] = obstacles[j].p1().x()/width();
        data[offset+4*j+1] = obstacles[j].p1().y()/width();
        data[offset+4*j+2] = obstacles[j].p2().x()/width();
        data[offset+4*j+3] = obstacles[j].p2().y()/width();
    }
    if(data.size()==0)return;
    vertexBuffer.write(0,data.data(),data.size()*sizeof(float));
    //fill indices
    QVector<uint> indices = QVector<uint>(2*object.size()*obstacles.size());
    indexBuffer.allocate(indices.size()*sizeof(uint));
    for(int i=0;i<object.size();++i){
        for(int j=0;j<obstacles.size();++j){
            indices[2*obstacles.size()*i+2*j+ 0] = i;
            indices[2*obstacles.size()*i+2*j+ 1] = object.size()+j;
        }
    }
    if(indices.size()==0)return;
    indexBuffer.write(0,indices.data(),indices.size()*sizeof(uint));

    glm::mat4 fullMat = projMat*viewportMat*worldMat;
    drawFull(fullMat, indices.size());
}

void StateGLWidget::drawFull(glm::mat4 fullMat, int drawSize)
{
    typedef GLfloat (*parr44)[4][4];

    shader_ev.bind();
    shader_ev.setUniformValue("fullMat",*(parr44)&fullMat);
    shader_ev.setAttributeBuffer("line0", GL_FLOAT,0*sizeof(float),4,4*sizeof(float));
    shader_ev.enableAttributeArray("line0");
    glDrawElements(GL_LINES, drawSize, GL_UNSIGNED_INT, 0);

    shader_ve.bind();
    shader_ve.setUniformValue("fullMat",*(parr44)&fullMat);
    shader_ve.setAttributeBuffer("line0", GL_FLOAT,0*sizeof(float),4,4*sizeof(float));
    shader_ve.enableAttributeArray("line0");
    glDrawElements(GL_LINES, drawSize, GL_UNSIGNED_INT, 0);

    shader_wires.bind();
    shader_wires.setUniformValue("fullMat",*(parr44)&fullMat);
    shader_wires.setAttributeBuffer("line0", GL_FLOAT,0*sizeof(float),4,4*sizeof(float));
    shader_wires.enableAttributeArray("line0");
    glDrawElements(GL_LINES, drawSize, GL_UNSIGNED_INT, 0);

    shader_wires.release();
}

void StateGLWidget::initShaderPrograms()
{
    bool result;
    result = initShaderProgram(shader_ev,{
                           {QOpenGLShader::Vertex,":/shaders/surf-ev/vsh.glsl"},
                           {QOpenGLShader::Geometry,":/shaders/surf-ev/gsh.glsl"},
                           {QOpenGLShader::Fragment,":/shaders/surf-ev/fsh.glsl"},
                       });
    if(!result){
        qWarning() << "Shader program warning: shader_ev";
    }

    result = initShaderProgram(shader_ve,{
                           {QOpenGLShader::Vertex,":/shaders/surf-ve/vsh.glsl"},
                           {QOpenGLShader::Geometry,":/shaders/surf-ve/gsh.glsl"},
                           {QOpenGLShader::Fragment,":/shaders/surf-ve/fsh.glsl"},
                       });
    if(!result){
        qWarning() << "Shader program warning: shader_ve";
    }

    result = initShaderProgram(shader_wires,{
                           {QOpenGLShader::Vertex,":/shaders/wire/vsh.glsl"},
                           {QOpenGLShader::Geometry,":/shaders/wire/gsh.glsl"},
                           {QOpenGLShader::Fragment,":/shaders/wire/fsh.glsl"},
                       });
    if(!result){
        qWarning() << "Shader program warning: shader_wires";
    }
}

bool StateGLWidget::initShaderProgram(QOpenGLShaderProgram&shader,
                                      QMap<QOpenGLShader::ShaderTypeBit,QString> sourceNames)
{
    bool result;
    for(auto&key:sourceNames.keys()){
        result = shader.addShaderFromSourceFile(key, sourceNames[key]);
        if(!result)
            qWarning() << shader.log();
    }

    result = shader.link();
    if(!result){
        qWarning() << "Could not link shProgram" << shader.log();
    }

    if(!shader.bind()){
        qWarning() << "Could not bind shProgram";
        return false;
    }

    return true;
}

bool StateGLWidget::initBuffers()
{
    if(!scene){
        qWarning() << "No scene available";
        //return false;
    }
    vertexBuffer.create();
    vertexBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    if(!vertexBuffer.bind()){
        qWarning() << "Could not bind vBuffer.";
        return false;
    }
    indexBuffer.create();
    indexBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    if(!indexBuffer.bind()){
        qWarning() << "Could not bind iBuffer.";
        return false;
    }
    return true;
}
