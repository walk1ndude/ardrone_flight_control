#include "rosobject.hpp"

#include <QImage>
#include <QDebug>
#include <QDir>

#include <sensor_msgs/image_encodings.h>

#define CAMERA_TOPIC "ardrone/image_raw"

#define FRONTAL_CAMERA 0
#define BOTTOM_CAMERA 1

#define SETCAMCHANNEL_SERVICE "ardrone/setcamchannel"

RosObject::RosObject(QObject *parent) : QObject(parent), it(nh) {
    subscribe();
    setServices();
    loadCalibData("/home/walkindude/cameraParams.yml");
    setChannel();
}

void RosObject::setChannel() {
    channel = 1;
}

void RosObject::subscribe() {
    ardrone_cam = it.subscribe(CAMERA_TOPIC, 1, &RosObject::getRosImage, this);
}

void RosObject::setServices(){
    client = nh.serviceClient<ardrone_autonomy::CamSelect>(SETCAMCHANNEL_SERVICE);
}

void RosObject::changeCamera() {
    channel = 1 - channel;
    srv.request.channel = channel;
    client.call(srv);
}

void RosObject::loadCalibData(const char * path) {

    cv::FileStorage fs(path,cv::FileStorage::READ);

    fs["cameraMatrixFront"] >> cameraMatrix[FRONTAL_CAMERA];
    fs["distCoeffsFront"] >> distortion[FRONTAL_CAMERA];

    fs["cameraMatrixBottom"] >> cameraMatrix[BOTTOM_CAMERA];
    fs["distCoeffsBottom"] >> distortion[BOTTOM_CAMERA];

    assert(!cameraMatrix[FRONTAL_CAMERA].empty() && !distortion[FRONTAL_CAMERA].empty() &&
           !cameraMatrix[BOTTOM_CAMERA].empty() && !distortion[BOTTOM_CAMERA].empty());
    fs.release();
}


void RosObject::getRosImage(const sensor_msgs::ImageConstPtr & msg) {
    qDebug() << time.elapsed();
    time.start();
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat cvImage = cv_ptr->image;

    QImage imageFromCV = mat2QImage(runCalibration(cvImage));

    if (channel == FRONTAL_CAMERA) {
        emit signalNewImageFront(imageFromCV);
    }
    else {
        emit signalNewImageBottom(imageFromCV);
    }
}

void RosObject::startRosLoop() {
    ros::spin();
}

void RosObject::endRosLoop() {
    ros::shutdown();
}

QImage RosObject::mat2QImage(const cv::Mat & image){
    // 1 channel
    if (image.type() == CV_8UC1) {
        QVector<QRgb> colorTable;
        for(quint16 i = 0; i < 256; i ++)
            colorTable.push_back(qRgb(i,i,i));
        const uchar * qImageBuffer = (const uchar *)image.data;
        QImage img(qImageBuffer,image.cols,image.rows,image.step,QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img;
    }

    // 3 channels
    else {
        const uchar * qImageBuffer = (const uchar *)image.data;
        QImage img(qImageBuffer,image.cols,image.rows,image.step,QImage::Format_RGB888);
        return img.rgbSwapped();
    }
    /* don't have other variants, only 1 or 3 channels per image */
}

cv::Mat RosObject::runCalibration(cv::Mat & image) {
    cv::Mat undistortedImage;
    cv::undistort(image,undistortedImage,cameraMatrix[channel],distortion[channel]);

    return undistortedImage;
}

