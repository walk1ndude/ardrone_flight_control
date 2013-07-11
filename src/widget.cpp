#include "widget.hpp"
#include "ui_widget.h"

#include <QDebug>

#define INTERVAL 1

Widget::Widget(QWidget *parent) : QWidget(parent), ui(new Ui::Widget) {
    ui->setupUi(this);
    createThreads();
    setTimer();
    makeConnections();
    launchThreads();
}

void Widget::createThreads() {
    rosThread = new QThread();
    rosObject = new RosObject();
    rosObject->moveToThread(rosThread);
}

void Widget::makeConnections() {
    //start ros loop
    connect(rosThread,SIGNAL(started()),rosObject,SLOT(startRosLoop()));

    // get image from ros thread
    connect(rosObject,SIGNAL(signalNewImageFront(QImage)),this,SLOT(setOnLabel(QImage)));
    connect(rosObject,SIGNAL(signalNewImageBottom(QImage)),this,SLOT(setOnLabel(QImage)));

    //exit ros loop, finish ros thread
    connect(this,SIGNAL(signalRosQuit()),rosObject,SLOT(endRosLoop()),Qt::DirectConnection);
    connect(rosThread,SIGNAL(finished()),rosThread,SLOT(deleteLater()));

    //change camera
    connect(timer,SIGNAL(timeout()),rosObject,SLOT(changeCamera()),Qt::DirectConnection);
}

void Widget::setTimer() {
    timer = new QTimer(this);
    timer->setInterval(INTERVAL);
}

void Widget::setOnLabel(const QImage & image) {
    ui->image->setPixmap(QPixmap::fromImage(image));
}

void Widget::launchThreads() {
    rosThread->start();
    timer->start();
}

void Widget::closeEvent(QCloseEvent * event) {
    emit signalRosQuit();
    event->accept();
}

Widget::~Widget() {
    delete rosObject;
    delete rosThread;
    delete ui;
}

