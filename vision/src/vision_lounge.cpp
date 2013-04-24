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

//Perspective details: Bottom of image at 4.5ft forward from Hokuyo
//Length of square on checkerboard grid: 3.9in

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

class ImageConverter
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;

bool USE_CVWINDOW;

public:
    ImageConverter() : it_(nh_)
    {
        image_pub_ = it_.advertise("vision/out", 1);
        image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &ImageConverter::imageCb, this);

        USE_CVWINDOW = false;

        namedWindow("Input Video");
        namedWindow("Processed Video");
        namedWindow("Binary Object Map");
    }

    ImageConverter(bool w) : it_(nh_)
    {
        image_pub_ = it_.advertise("vision/out", 1);
        image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &ImageConverter::imageCb, this);

        USE_CVWINDOW = w;

        namedWindow("Input Video");
        namedWindow("Blur Video");
        namedWindow("Hue Video");
        namedWindow("Canny Video");
        namedWindow("Luminosity Video");
        namedWindow("Binary Object Map");
    }

    ~ImageConverter()
    {
        destroyWindow("Input Video");
        destroyWindow("Blur Video");
        destroyWindow("Hue Video");
        destroyWindow("Canny Video");
        destroyWindow("Luminosity Video");
        destroyWindow("Binary Object Map");
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        //Main processing node

        gpu::GpuMat gcarpetcolor;
    
        //Convert input into usable arguments
        Mat src;        
        try
        {
            src = cv_bridge::toCvCopy(msg, enc::BGR8)->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        

        gpu::GpuMat gSrc = (gpu::GpuMat) src;
        gpu::GpuMat gGray;
        gpu::GpuMat gblur_image;
        gpu::GpuMat gproc_image;
        gpu::GpuMat ghsv_image;


        //cv::Rect ROI = cv::Rect(0, 0, gSrc.cols, (gSrc.rows)*6/7);
        //gSrc = gpu::GpuMat(gSrc, ROI);

        

        if (USE_CVWINDOW) {
        imshow("Input Video",(Mat)gSrc);
        waitKey(30); }

        gpu::GaussianBlur(gSrc, gblur_image, Size(11,11), 5, 5);
        gpu::cvtColor(gblur_image, gproc_image, CV_BGR2HLS);
        gpu::cvtColor(gblur_image, ghsv_image, CV_BGR2HSV);
        gpu::cvtColor(gblur_image, gGray, CV_BGR2GRAY);

        if (USE_CVWINDOW) {
        imshow("Blur Video",(Mat)gblur_image);
        waitKey(30); }

/*
        //PROCESSING TIME!! YAAAY~~~
        //Following Frank's steps so far (this is his ported code)
        //EDITED: Cut blur from 7 to 5 to prevent grass from bleeding over thin white lines
        gpu::GaussianBlur(gSrc, gblur_image, Size(13,13), 5, 5);
        gpu::cvtColor(gblur_image, gproc_image, CV_BGR2HSV);

        gpu::cvtColor(gSrc, gGray, CV_BGR2GRAY);


        imshow("Processed Video", (Mat) gblur_image);
        waitKey(30);

        //Split into HSV channels and RGB channels too
        //Using Frank names and prepending g to all gpu mats

        vector<gpu::GpuMat> gsplit_bgr;
        gpu::split(gblur_image, gsplit_bgr);        

        vector<gpu::GpuMat> gsplit_image;
        gpu::split(gproc_image, gsplit_image);
        gpu::GpuMat gthresh_0, gthresh_1, gthresh_2, gthresh_3;

        gpu::threshold(gsplit_image[1], gthresh_0, 90, 255, THRESH_BINARY);// >x% saturation
        gpu::threshold(gsplit_image[2], gthresh_3, 60, 255, THRESH_BINARY); // >y% Value
        gpu::threshold(gsplit_image[0], gthresh_1, 100, 140, THRESH_BINARY);// < purple


        gpu::add(gthresh_1, gthresh_0, gcarpetcolor);
        gpu::add(gthresh_3, gcarpetcolor, gcarpetcolor);
        gpu::bitwise_and(gcarpetcolor, gthresh_0, gcarpetcolor);


        ///Begin THIRD FLOOR CARPET detection

        //gpu::cvtColor(gblur_image, gproc_image, CV_BGR2HLS);
        //gpu::split(gproc_image, gsplit_image);

        //double min = 0;
        //double max = 0;
        //gpu::minMax(gsplit_image[1], &min, &max);
        
        
        //gpu::threshold(gsplit_image[1], gthresh_0, (int)(max * 0.75), 255, THRESH_BINARY);



        //Fiddle around with the shape of the kernels to make it detect vertical white lines well
        gpu::GpuMat closed;
        gpu::erode(gcarpetcolor, closed, Mat::ones(5, 5, CV_8U));
        gpu::dilate(closed, gcarpetcolor, Mat::ones(5, 5, CV_8U));
        
*/

        /*Mat destination;

        gpu::meanShiftSegmentation(gproc_image, destination, 20, 20, 50);

        cvtColor(destination, destination, CV_RGBA2BGR);
        cvtColor(destination, destination, CV_BGR2HLS);

        vector<Mat> gsplit_image;
        split(destination, gsplit_image);

        CvScalar temp = mean(gsplit_image[0]);
        int avghue = temp.val[0];
        inRange((Mat)gsplit_image[0], avghue-10, avghue+10, destination);
        bitwise_not(destination, destination);*/

        vector<gpu::GpuMat> gsplit_image;
        gpu::split(gproc_image, gsplit_image);
        gpu::GpuMat gthresh_0, gthresh_1, gthresh_2, gthresh_3;


        gpu::threshold(gsplit_image[0], gthresh_1, 90, 135, THRESH_BINARY);
        gpu::threshold(gsplit_image[1], gthresh_0, 55, 255, THRESH_BINARY); //Sat normally 75 low
        gpu::threshold(gsplit_image[2], gthresh_3, 30, 255, THRESH_BINARY);
        

        if (USE_CVWINDOW) {
        imshow("Hue Video",(Mat)gthresh_1);
        waitKey(30); }

        

        if (USE_CVWINDOW) {
        imshow("Luminosity Video",(Mat)gthresh_3);
        waitKey(30); }

        gpu::add(gthresh_1, gthresh_0, gcarpetcolor);
        gpu::add(gthresh_3, gcarpetcolor, gcarpetcolor);
        //gpu::bitwise_and(gthresh_3, gcarpetcolor, gcarpetcolor);
        gpu::bitwise_and(gcarpetcolor, gthresh_0, gcarpetcolor);
        bitwise_not(gcarpetcolor, gcarpetcolor);


        gpu::split(ghsv_image, gsplit_image);
        gpu::GpuMat gred_orange;

        gpu::threshold(gsplit_image[2], gthresh_3, 210, 255, THRESH_BINARY);
        gpu::threshold(gsplit_image[1], gthresh_0, 80, 255, THRESH_BINARY);
        gpu::threshold(gsplit_image[0], gthresh_1, 210, 255, THRESH_BINARY);
        gpu::threshold(gsplit_image[0], gthresh_2,  40, 255, THRESH_BINARY_INV);

        gpu::add(gthresh_1, gthresh_2, gred_orange);
        gpu::add(gred_orange, gthresh_3, gred_orange);
        gpu::bitwise_and(gred_orange, gthresh_0, gred_orange);

        gpu::add(gred_orange, gcarpetcolor, gcarpetcolor);

        ///////////////////////////////////////
        gpu::GpuMat cannyEdges;
        gpu::GpuMat dilated;
        gpu::Canny( gGray, cannyEdges, 17, 17*3, 3 );
    
        gpu::dilate(cannyEdges, dilated, Mat::ones(3, 3, CV_8U));

        if (USE_CVWINDOW) {
        imshow("Canny Video",(Mat)dilated);
        waitKey(30); }

        gpu::add(dilated, gcarpetcolor, gcarpetcolor);
        ///////////////////////////////////////


        Mat mask = (Mat) gcarpetcolor;

        rectangle( mask,
           Point( mask.cols/3.0, mask.rows/**(14.0/15)*/ ),
           Point( mask.cols*2.0/3.0, mask.cols),
           Scalar( 0, 0, 0 ), CV_FILLED );

        gcarpetcolor = (gpu::GpuMat) mask;

        gpu::threshold(gcarpetcolor, gcarpetcolor, 100,255, THRESH_BINARY);

       

        if (USE_CVWINDOW) {
        imshow("Binary Object Map", (Mat) gcarpetcolor);
        waitKey(30); }


        //Publish image results
        Mat outMat = (Mat)gcarpetcolor;
        cv_bridge::CvImage out_msg;
        out_msg.header   = msg->header;
        out_msg.encoding = enc::MONO8;
        out_msg.image    = outMat;

        image_pub_.publish(out_msg.toImageMsg());
    }


};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "image_converter");

    /*if (argc == 1)
    {
        if (strcmp(argv[0],"-w") == 0)
            ImageConverter ic(true);
    }
    
    else {
        ImageConverter ic(false);
    }*/

    ImageConverter ic(true);

    ros::spin();

    return 0;
}
