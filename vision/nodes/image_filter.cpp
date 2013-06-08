#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <cstring>
#include <fstream>

namespace enc = sensor_msgs::image_encodings;
//class denoise = cv::gpu::FastNonLocalMeansDenoising;
using namespace std;
using namespace cv;

class Filter
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Publisher image_pub_;
image_transport::Subscriber image_sub_;
bool SHOW_IMAGES;
//gpu::FastNonLocalMeansDenoising denoise_;
public:
    Filter(bool show) : it_(nh_){
        image_pub_ = it_.advertise("vision/filtered", 1);
        image_sub_ = it_.subscribe("usb_cam/image_raw" , 1, &Filter::process, this);
        SHOW_IMAGES = show;
        if (SHOW_IMAGES) {
            namedWindow("Input Video");
            namedWindow("Output Video");
        }
    }

    ~Filter()
    {
        if (SHOW_IMAGES) {
            destroyWindow("Input Video");
            destroyWindow("Output Video");
        }
    }
    
    void process(const sensor_msgs::ImageConstPtr& msg)
    {
        Mat src, dest;
        try
        {
            src = cv_bridge::toCvCopy(msg, enc::BGR8)->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        if (SHOW_IMAGES) {
            imshow("Input Video", src);
            waitKey(30); 
        }
        
        dest = processImageGPU(src);
        
        if (SHOW_IMAGES) {
            imshow("Output Video", dest);
            waitKey(30); 
        }
        
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = enc::BGR8;
        out_msg.image = dest;

        image_pub_.publish(out_msg.toImageMsg());
    }
    
    Mat processImageGPU(Mat src)
    {
        gpu::GpuMat gSrc = (gpu::GpuMat) src;
        vector<gpu::GpuMat> gHSV_split;
        gpu::GpuMat gProc, gProc1, gOut;
        
        gpu::FastNonLocalMeansDenoising.simpleMethod(gSrc, gOut, 10.0f, 21, 7);

        gpu::cvtColor(gOut,gProc,CV_BGR2HSV);
        gpu::split(gProc,gHSV_split);
        gpu::equalizeHist(gHSV_split[2], gHSV_split[2]);
        gpu::merge(gHSV_split,gProc);
        gpu::cvtColor(gProc,gOut,CV_HSV2BGR);

        return (Mat)gOut;
    }
    
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, ("image_filter"));
    ros::NodeHandle nh("~");
    bool show;
    nh.param<bool>("show",show,false);
    Filter f(show);
    ros::spin();

    return 0;
}

