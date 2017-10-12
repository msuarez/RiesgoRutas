#-------------------------------------------------
#
# Project created by QtCreator 2015-11-05T15:40:28
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = vision_3d
TEMPLATE = app

LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_calib3d -lopencv_features2d

SOURCES += main.cpp\
        mainwindow.cpp \
    calibratesetting.cpp \
    calibratecam.cpp

HEADERS  += mainwindow.h \
    calibratesetting.h \
    calibratecam.h

FORMS    += mainwindow.ui \
    calibratesetting.ui \
    calibratecam.ui
