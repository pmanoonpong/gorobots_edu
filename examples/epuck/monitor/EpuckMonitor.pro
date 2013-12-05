#-------------------------------------------------
#
# Project created by QtCreator 2012-05-16T18:06:02
#
#-------------------------------------------------

QT       += core gui widgets

TARGET = EpuckMonitor
TEMPLATE = app

LPZROBOTS = $$system("ode_robots-config --srcprefix")/..
SELFORGLIBFILE = $$system(selforg-config --libfile)

FORMS    += \
    monitor.ui

SOURCES += \
   main.cpp\
   monitor.cpp \
   plotwidget.cpp \
   $$LPZROBOTS/real_robots/robots/epuck/SerialComm.cpp \
   $$LPZROBOTS/real_robots/robots/epuck/epuckbluetooth.cpp

HEADERS  += \
    monitor.h \
    plotwidget.h \
    $$LPZROBOTS/real_robots/robots/epuck/SerialComm.h \
    $$LPZROBOTS/real_robots/robots/epuck/epuckbluetooth.h

INCLUDEPATH += \
    /home/timo/workspace/lpzrobots/real_robots/robots/epuck \
    /home/timo/workspace/lpzrobots/selforg/include \
    $$LPZROBOTS/real_robots/robots/epuck \
    $$LPZROBOTS/selforg/include

LIBS       += -lreadline \
              -lncurses \
              $$SELFORGLIBFILE
