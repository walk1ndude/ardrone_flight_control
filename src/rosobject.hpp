#ifndef ROSOBJECT_H
#define ROSOBJECT_H

#include <QObject>
#include <QImage>

#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class RosObject : public QObject
{
    Q_OBJECT
public:
    explicit RosObject(QObject * parent = 0);

private:
    ros::NodeHandle nh;
    ros::ServiceClient client;

    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub_f;
    image_transport::Subscriber image_sub_b;

    cv_bridge::CvImagePtr cv_ptr;

    cv::Mat cameraMatrix;
    cv::Mat distortion;

    void subscribe();
    void loadCalibData(const char * path);
    void getRosImage(const sensor_msgs::ImageConstPtr & msg);

    QImage mat2QImage(const cv::Mat &image);

    cv::Mat runCalibration(cv::Mat & image);

signals:
    void signalNewImage(QImage);

public slots:
    void startLoop();
};

#endif // ROSTREAD_H
