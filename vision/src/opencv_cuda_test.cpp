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
#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

void test_cuda() {
    try {
        cv::Mat src_host = cv::imread("TESTIMAGE.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        cv::gpu::GpuMat dst, src;
        src.upload(src_host);

        cv::gpu::threshold(src, dst, 128.0, 255.0, CV_THRESH_TOZERO);

        cv::Mat result_host = (cv::Mat) dst;
        
        std::cout << "cuda works!" << std::endl;

        cv::imshow("Result", result_host);
        cv::waitKey();
    } catch(const cv::Exception& ex) {
        std::cout << "Error: " << ex.what() << std::endl;
    }
}

void test_opencv() {
    try {
        cv::Mat dst,
                src = cv::imread("TESTIMAGE.jpg", CV_LOAD_IMAGE_GRAYSCALE);

        cv::threshold(src, dst, 128.0, 255.0, CV_THRESH_TOZERO);
        
        std::cout << "opencv works!" << std::endl;

        cv::imshow("Result", dst);
        cv::waitKey();
    } catch(const cv::Exception& ex) {
        std::cout << "Error: " << ex.what() << std::endl;
    }
}

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "opencv_cuda_test");
    
    test_cuda();
    test_opencv();
}


