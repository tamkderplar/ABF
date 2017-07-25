#-------------------------------------------------
#
# Project created by QtCreator 2016-11-04T01:24:15
#
#-------------------------------------------------

QT       += core gui opengl

CONFIG  += c++14

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ABF
TEMPLATE = app

INCLUDEPATH += widgets/ views/

SOURCES += main.cpp\
        mainwindow.cpp \
    freespaceboundary.cpp \
    bfpface.cpp \
    graph.cpp \
    widgets/stateglwidget.cpp \
    widgets/stateglwidget_input.cpp \
    widgets/grapheditor.cpp \
    widgets/thetaslicewidget.cpp \
    widgets/contactwidget.cpp \
    widgets/scenewidget.cpp \
    views/thetasliceview.cpp \
    doublecontactfunction.cpp \
    treeforqpointf.cpp \
    misc/range.cpp \
    restrictedregion.cpp

HEADERS  += mainwindow.h \
    glm.h \
    freespaceboundary.h \
    bfpface.h \
    graph.h \
    widgets/stateglwidget.h \
    widgets/grapheditor.h \
    widgets/thetaslicewidget.h \
    widgets/contactwidget.h \
    widgets/scenewidget.h \
    views/thetasliceview.h \
    doublecontactfunction.h \
    misc/itertools.hpp \
    treeforqpointf.h \
    misc/range.h \
    misc/misc.h \
    restrictedregion.h

FORMS    += mainwindow.ui \
    views/thetasliceview.ui

DISTFILES += \
    sh-ev-fsh.glsl \
    sh-ev-gsh.glsl \
    sh-ev-vsh.glsl \
    sh-ve-fsh.glsl \
    sh-ve-gsh.glsl \
    sh-ve-vsh.glsl \
    sh-wire-gsh.glsl \
    sh-wire-vsh.glsl

RESOURCES += \
    shaders.qrc

test {
    message(Test build)
    QT += testlib
    TARGET = UnitTests

    SOURCES -= main.cpp

    HEADERS += test/testdcf.h

    SOURCES += test/main.cpp
} else {
    message(Normal build)
}
