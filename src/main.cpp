#include "widget.hpp"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc,argv,"droneui");
    Widget w;
    w.show();
    
    return a.exec();
}
