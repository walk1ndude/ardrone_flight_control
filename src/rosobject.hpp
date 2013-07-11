#ifndef ROSOBJECT_H
#define ROSOBJECT_H

#include <QObject>
#include <QImage>
#include <QTime>

#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <ardrone_autonomy/CamSelect.h>

class RosObject : public QObject
{
    Q_OBJECT
public:
    explicit RosObject(QObject * parent = 0);

private:
    ros::NodeHandle nh;
    ros::ServiceClient client;

    image_transport::ImageTransport it;
    image_transport::Subscriber ardrone_cam;

    cv_bridge::CvImagePtr cv_ptr;

    cv::Mat cameraMatrix[2];
    cv::Mat distortion[2];

    QTime time;
    bool measured;

    ardrone_autonomy::CamSelect srv;

    int channel;

    void setChannel();
    void subscribe();
    void setServices();
    void loadCalibData(const char * path);
    void getRosImage(const sensor_msgs::ImageConstPtr & msg);

    QImage mat2QImage(const cv::Mat &image);

    cv::Mat runCalibration(cv::Mat & image);

signals:
    void signalNewImageFront(QImage);
    void signalNewImageBottom(QImage);

public slots:
    void startRosLoop();
    void endRosLoop();
    void changeCamera();
};

#endif // ROSTREAD_H
