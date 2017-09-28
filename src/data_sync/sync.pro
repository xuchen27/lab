#-------------------------------------------------
#
# Project created by QtCreator 2017-09-23T13:27:48
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = sync_node
CONFIG   += console
CONFIG   -= app_bundle

HEADERS += ./include/data_sync/*.h \

TEMPLATE = app

SOURCES += src/*.cxx \


INCLUDEPATH += /usr/local/include \
               /usr/local/include/serial_port \
               /opt/ros/kinetic/include

LIBS += /usr/local/lib/libserial_port.so \


