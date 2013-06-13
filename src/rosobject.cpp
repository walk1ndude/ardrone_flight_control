#include "rosobject.hpp"

#include <QImage>
#include <QDebug>
#include <Qt>

#include <sensor_msgs/image_encodings.h>

#define FRONTAL_CAMERA_TOPIC "ardrone/front/image_raw"
#define BOTTOM_CAMERA_TOPIC "ardrone/bottom/image_raw"

RosObject::RosObject(QObject *parent) : QObject(parent), it(nh) {
    subscribe();
    loadCalibData("/home/walkindude/cameraParams.yml");
}

void RosObject::subscribe() {
    image_sub_f = it.subscribe(FRONTAL_CAMERA_TOPIC, 1, &RosObject::getRosImage, this);
    image_sub_b = it.subscribe(BOTTOM_CAMERA_TOPIC, 1, &RosObject::getRosImage, this);
}

void RosObject::loadCalibData(const char * path) {

    cv::FileStorage fs(path,cv::FileStorage::READ);

    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distortion;

    assert(!cameraMatrix.empty() && !distortion.empty());
    fs.release();
}


void RosObject::getRosImage(const sensor_msgs::ImageConstPtr & msg) {
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    emit signalNewImage(mat2QImage(runCalibration(cv_ptr->image)));
}

void RosObject::startLoop() {
    ros::spin();
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

    cv::undistort(image,undistortedImage,cameraMatrix,distortion);

    return undistortedImage;
}

