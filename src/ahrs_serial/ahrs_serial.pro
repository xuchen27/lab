#-------------------------------------------------
#
# Project created by QtCreator 2016-03-16T11:30:48
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = structured_light
CONFIG   += console
CONFIG   -= app_bundle

HEADERS += ./include/ahrs_serial/*.h \

TEMPLATE = app

SOURCES += src/*.cxx \
   

INCLUDEPATH += /usr/local/include \
               /usr/local/include/serial_port \
               /opt/ros/kinetic/include

LIBS += /usr/local/lib/libserial_port.so \


