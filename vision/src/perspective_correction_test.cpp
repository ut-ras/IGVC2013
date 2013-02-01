#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <cv.h>
//#include <highgui.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

namespace enc = sensor_msgs::image_encodings;

int EXPECTED_WIDTH = 640,
    WINDOW_HEIGHT = 30;

cv::Size boardSize(3,3);

void dispImages(cv::Mat img) {
    std::vector<cv::Point2f> corners;
    
    bool found = cv::findChessboardCorners(
        img,
        boardSize,	
        corners,
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE
    );

    if (!found) {
        ROS_ERROR("ERROR: cannot find chess board in image");
        return;
    }

    cv::Mat transformed;
    // transformImage(img, transformed, &corners, boardSize);

    cv::drawChessboardCorners(img, boardSize, cv::Mat(corners), found);

    cv::imshow("Plain", img);
    // cv::imshow("Transformed", transformed);

    cv::waitKey(30);
}

void callback(const sensor_msgs::ImageConstPtr& msg) {
    ROS_INFO("processing image!");

    cv::Mat img;
    try {
        img = cv_bridge::toCvCopy(msg, enc::BGR8)->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    dispImages(img);
}

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "perspective_correction_test");

    cv::namedWindow("Plain");
    cv::moveWindow("Plain", 0, WINDOW_HEIGHT);

    cv::namedWindow("Transformed");
    cv::moveWindow("Transformed", EXPECTED_WIDTH, WINDOW_HEIGHT);

    ROS_INFO("Now, move the chessboard around to get a correct perspective transformation!");
    
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("usb_cam/image_raw", 1, callback);

    ros::spin();
}


