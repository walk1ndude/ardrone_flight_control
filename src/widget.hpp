#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QThread>

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

    void createThreads();
    void makeConnections();
    void launchThreads();

signals:
    void signalRosQuit();

public slots:
    void setOnLabel(const QImage & image);
    void testClicked();

};

#endif // WIDGET_H
