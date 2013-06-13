#include "widget.hpp"
#include "ui_widget.h"

#include <QDebug>

Widget::Widget(QWidget *parent) : QWidget(parent), ui(new Ui::Widget) {
    ui->setupUi(this);
    createThreads();
    makeConnections();
    launchThreads();
}

void Widget::createThreads() {
    rosThread = new QThread();
    rosObject = new RosObject();
    rosObject->moveToThread(rosThread);
}

void Widget::makeConnections() {
    connect(rosThread,SIGNAL(started()),rosObject,SLOT(startLoop()));
    connect(rosObject,SIGNAL(signalNewImage(QImage)),this,SLOT(setOnLabel(QImage)));
    connect(ui->button,SIGNAL(clicked()),this,SLOT(testClicked()));
    connect(rosThread,SIGNAL(finished()),rosThread,SLOT(deleteLater()));
}

void Widget::setOnLabel(const QImage & image) {
    ui->image->setPixmap(QPixmap::fromImage(image));
}

void Widget::testClicked() {
    qDebug() << "test, gui's not frozen";
}

void Widget::launchThreads() {
    rosThread->start();
}

Widget::~Widget() {
    delete rosObject;
    delete rosThread;
    delete ui;
}
