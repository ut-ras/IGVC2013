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

class Features
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Publisher image_pub_;
image_transport::Subscriber image_sub_;
bool SHOW_IMAGES;
public:
    Features(): it_(nh_){
        Features(true);
    }
    
    Features(bool w) : it_(nh_)
    {
        image_pub_ = it_.advertise("vision/canny", 1);
        image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &Features::callback , this);
        SHOW_IMAGES = w;
        if (SHOW_IMAGES) {
            namedWindow("Input Video");
            namedWindow("Output Video");
        }
    }

    ~Features()
    {
        if (SHOW_IMAGES) {
            destroyWindow("Input Video");
            destroyWindow("Output Video");
        }
    }
    
    void callback(const sensor_msgs::ImageConstPtr& msg)
    {
        Mat src, dest;
        ros::Time stime,etime;
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
        
        dest = processImage(src);
        
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
    
    Mat processImage(Mat src)
    {
        gpu::GpuMat gSrc = (gpu::GpuMat) src;
        gpu::GpuMat gGray, gBlur, cannyEdges, dilated;
        gpu::GaussianBlur(gSrc, gBlur, Size(11,11), 5, 5);
        gpu::cvtColor(gBlur, gGray, CV_BGR2GRAY);
        gpu::Canny( gGray, cannyEdges, 17, 17*3, 3 );
        gpu::dilate(cannyEdges, dilated, Mat::ones(3, 3, CV_8U));
        return (Mat)dilated;
    }
    
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "CPU_vs_GPU");

    Features f(true);

    ros::spin();

    return 0;
}

