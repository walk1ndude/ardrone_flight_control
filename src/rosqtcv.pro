#-------------------------------------------------
#
# Project created by QtCreator 2013-06-11T18:47:14
#
#-------------------------------------------------

greaterThan(QT_MAJOR_VERSION, 4): QT += core gui widgets

TARGET = rosqtcv
TEMPLATE = app


SOURCES += main.cpp\
        widget.cpp \
        rosobject.cpp

HEADERS  += widget.hpp \
            rosobject.hpp

FORMS    += ../ui/widget.ui

INCLUDEPATH += /opt/ros/groovy/include \
    /home/walkindude/catkin_ws/ardrone_autonomy/msg_gen/cpp/include \
    /home/walkindude/catkin_ws/tum_ardrone/msg_gen/cpp/include

LIBS += -L /opt/ros/groovy/lib -lopencv_core \
    -lopencv_imgproc \
    -lroscpp \
    -lrosconsole \
    -lrostime \
    -lroscpp_serialization \
    -lxmlrpcpp \
    -limage_transport \
    -lmessage_filters \
    -lclass_loader \
    -lconsole_bridge \
    -lroslib \
    -lrospack

RESOURCES += \
    ../resources/resources.qrc
