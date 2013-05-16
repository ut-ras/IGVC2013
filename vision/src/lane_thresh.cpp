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

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

class LaneThresholder
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Publisher image_pub_;
image_transport::Subscriber image_sub_;
bool SHOW_IMAGES;
public:
    LaneThresholder(): it_(nh_){
        LaneThresholder(true);
    }
    
    LaneThresholder(bool w) : it_(nh_)
    {
        image_pub_ = it_.advertise("vision/lane_thresh", 1);
        image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &LaneThresholder::process, this);
        SHOW_IMAGES = w;
        if (SHOW_IMAGES) {
            namedWindow("Input Video");
            namedWindow("Output Video");
        }
    }

    ~LaneThresholder()
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
        out_msg.encoding = enc::MONO8;
        out_msg.image = dest;

        image_pub_.publish(out_msg.toImageMsg());
    }
    
    Mat processImageGPU(Mat src)
    {
        gpu::GpuMat gSrc = (gpu::GpuMat) src;
        gpu::GpuMat gHSV, gProc, gOut;
        vector<gpu::GpuMat> gBGR_split, gHSV_split;
        gpu::cvtColor(gSrc, gHSV, CV_BGR2HSV);
        gpu::split(gSrc,gBGR_split);
        gpu::split(gHSV,gHSV_split);
        gpu::threshold(gHSV_split[0], gOut, 103, 112, THRESH_BINARY);
        gpu::threshold(gBGR_split[0], gProc, 250, 255, THRESH_BINARY);
        gpu::bitwise_and(gOut, gProc, gOut);
        return (Mat)gOut;
    }
    
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Lane_Thresholder");

    LaneThresholder t(false);

    ros::spin();

    return 0;
}

