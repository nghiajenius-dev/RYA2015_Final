#-------------------------------------------------
#
# Project created by QtCreator 2016-08-30T17:10:17
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = rya2015-opencv3
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

# Tell the compiler where to look for libippicv.a
LIBS += -L/usr/local/Cellar/opencv3/3.1.0_3/share/OpenCV/3rdparty/lib

# If simplink libippicv to /usr/local/lib
#LIBS += -L/usr/local/lib


# The following lines tells Qmake to use pkg-config for opencv
QT_CONFIG -= no-pkg-config
CONFIG  += link_pkgconfig
PKGCONFIG += opencv

