#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"
//#include <cv.h>
//#include <highgui.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

class Thresholder
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::ImageTransport image_sub_;
image_transport::ImageTransport image_pub_;

public:
    Thresholder() : it_(nh_) {

        image_pub_ = it.advertise("vision/thresholded", 1);
        image_sub_ = it.subscribe("usb_cam/image_raw", 1, &Thresholder::apply_threshold, this);

        namedWindow("RAW");
        namedWindow("OUTPUT");
    }

    ~Thresholder() {
        destroyWindow("RAW");
        destroyWindow("OUTPUT");
    }

    void apply_threshold(cont sensor_msgs::ImageConstPtr& msg) {
        gpu::GpuMat

int main(int argc, char* argv[]) {
    using namespace ros

    init(argc, argv, "threshold_applicator");

    ImageConverter ic;
    spin();

    return 0;
}
