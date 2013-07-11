#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QThread>
#include <QCloseEvent>
#include <QTimer>

#include "rosobject.hpp"

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT
    
public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();
    
private:
    Ui::Widget *ui;

    QThread * rosThread;

    RosObject * rosObject;

    QTimer * timer;

    void setTimer();

    void createThreads();
    void makeConnections();
    void launchThreads();

protected:
    void closeEvent(QCloseEvent * event);

signals:
    void signalRosQuit();

public slots:
    void setOnLabel(const QImage & image);
};

#endif // WIDGET_H
