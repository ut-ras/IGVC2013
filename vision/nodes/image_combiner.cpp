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

#define MAX_VAL 255
#define SVBGR_MAX 255
#define H_MAX 180
#define BOOL 1
#define DEFAULT_THRESHOLDS {H_MAX, 0, 0, SVBGR_MAX, 0, SVBGR_MAX, 0, SVBGR_MAX, 0, SVBGR_MAX, 0, SVBGR_MAX, 0}
#define THRESHOLD_PATH "/home/ras/ros/ros-pkg/IGVC2013/vision/thresholds/"

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

class Thresholder
{

gpu::GpuMat hough_lines;
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Publisher image_pub_;
image_transport::Subscriber image_sub_;
image_transport::Subscriber image_sub2_;
bool SHOW_IMAGES;
bool SHOW_MAKER;
bool hough_lines_seen;
int thresholds[13];
public:
    Thresholder(ros::NodeHandle nh_) : it_(nh_){
        Thresholder(false, true);
    }
    
    Thresholder(bool show, bool maker) : it_(nh_){
        int thresh[] = DEFAULT_THRESHOLDS;
        Thresholder(show, maker, thresh, "maker");
    }    
    
    Thresholder(bool show, bool maker, int* input_thresholds, string name) : it_(nh_)
    {
        Thresholder(show,maker,input_thresholds,name,"usb_cam/image_raw");
    }
    Thresholder(bool show, bool maker, int* input_thresholds, string name, string input) : it_(nh_)
    {
        image_pub_ = it_.advertise("vision/thresholder_"+name, 1);
        image_sub_ = it_.subscribe(input , 1, &Thresholder::process, this);
        image_sub2_ = it_.subscribe("vision/hough_lines" , 1, &Thresholder::houghLinesProcess, this);
        SHOW_IMAGES = show;
        SHOW_MAKER = maker;
        hough_lines_seen = false;

        for(int i = 0; i < 13; i++) thresholds[i] = input_thresholds[i];
        if(SHOW_MAKER){
            namedWindow("Thresholder");
            createTrackbar("H+","Thresholder",&thresholds[0], H_MAX);
            createTrackbar("H-","Thresholder",&thresholds[1], H_MAX);
            createTrackbar("H~","Thresholder",&thresholds[2], BOOL);
            createTrackbar("S+","Thresholder",&thresholds[3], SVBGR_MAX);
            createTrackbar("S-","Thresholder",&thresholds[4], SVBGR_MAX);
            createTrackbar("V+","Thresholder",&thresholds[5], SVBGR_MAX);
            createTrackbar("V-","Thresholder",&thresholds[6], SVBGR_MAX);
            createTrackbar("B+","Thresholder",&thresholds[7], SVBGR_MAX);
            createTrackbar("B-","Thresholder",&thresholds[8], SVBGR_MAX);
            createTrackbar("G+","Thresholder",&thresholds[9], SVBGR_MAX);
            createTrackbar("G-","Thresholder",&thresholds[10], SVBGR_MAX);
            createTrackbar("R+","Thresholder",&thresholds[11], SVBGR_MAX);
            createTrackbar("R-","Thresholder",&thresholds[12], SVBGR_MAX);
        }
        if (SHOW_IMAGES) {
            namedWindow("Input Video");
            namedWindow("Output Video");
        }
    }

    ~Thresholder()
    {
        if(SHOW_MAKER){
            destroyWindow("Thesholder");
            printf("\n");
        }
        if (SHOW_IMAGES) {
            destroyWindow("Input Video");
            destroyWindow("Output Video");
        }
    }

    void houghLinesProcess(const sensor_msgs::ImageConstPtr& msg)
    {
        gpu::GpuMat lines;
        try
        {
           lines = (gpu::GpuMat) cv_bridge::toCvCopy(msg, enc::BGR8)->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        vector<gpu::GpuMat> channel_split;
        gpu::split(lines, channel_split);
        hough_lines = channel_split[0];
        hough_lines_seen = true;
    }

    void process(const sensor_msgs::ImageConstPtr& msg)
    {
        Mat src;
        gpu::GpuMat dest;

        cout << "HI!!!" << endl;
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
        if (hough_lines_seen) {
            gpu::bitwise_or(dest, hough_lines, dest);
        }
        
        if(SHOW_MAKER){
            printf("Thresholds: ");
            for(int i = 0; i<13;i++){
                printf("%d ", thresholds[i]);
            }
            printf("\r");
            imshow("Thesholder",dest);
            waitKey(30);
        }

        if (SHOW_IMAGES) {
            imshow("Output Video", dest);
            waitKey(30); 
        }
        
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = enc::TYPE_8UC1;
        out_msg.image = (Mat)dest;

        image_pub_.publish(out_msg.toImageMsg());
    }
    
    gpu::GpuMat processImageGPU(Mat src)
    {
        gpu::GpuMat gSrc = (gpu::GpuMat) src;
        gpu::GpuMat gHSV, gProc, gProc2, gOut;
        vector<gpu::GpuMat> gBGR_split, gHSV_split;
        
        gpu::cvtColor(gSrc, gHSV, CV_BGR2HSV);
        gpu::split(gSrc,gBGR_split);
        gpu::split(gHSV,gHSV_split);
        
        gpu::threshold(gHSV_split[0], gOut, thresholds[0], MAX_VAL, (!thresholds[2])?THRESH_BINARY_INV:THRESH_BINARY);
        gpu::threshold(gHSV_split[0], gProc, thresholds[1], MAX_VAL , thresholds[2]?THRESH_BINARY_INV:THRESH_BINARY);
        if(thresholds[2]) gpu::bitwise_or(gOut, gProc, gOut); else gpu::bitwise_and(gOut, gProc, gOut);
        
        gpu::threshold(gHSV_split[1], gProc, thresholds[3], MAX_VAL , THRESH_BINARY_INV);
        gpu::threshold(gHSV_split[1], gProc2, thresholds[4], MAX_VAL ,THRESH_BINARY);
        gpu::bitwise_and(gProc, gProc2, gProc);
        gpu::bitwise_and(gOut, gProc, gOut);
        
        gpu::threshold(gHSV_split[2], gProc, thresholds[5],  MAX_VAL , THRESH_BINARY_INV);
        gpu::threshold(gHSV_split[2], gProc2, thresholds[6],  MAX_VAL , THRESH_BINARY);
        gpu::bitwise_and(gProc, gProc2, gProc);
        gpu::bitwise_and(gOut, gProc, gOut);
        
        gpu::threshold(gBGR_split[0], gProc, thresholds[7], MAX_VAL , THRESH_BINARY_INV);
        gpu::threshold(gBGR_split[0], gProc2, thresholds[8], MAX_VAL , THRESH_BINARY);
        gpu::bitwise_and(gProc, gProc2, gProc);
        gpu::bitwise_and(gOut, gProc, gOut);
        
        gpu::threshold(gBGR_split[1], gProc, thresholds[9], MAX_VAL, THRESH_BINARY_INV);
        gpu::threshold(gBGR_split[1], gProc2, thresholds[10], MAX_VAL , THRESH_BINARY);
        gpu::bitwise_and(gProc, gProc2, gProc);
        gpu::bitwise_and(gOut, gProc, gOut);
        
        gpu::threshold(gBGR_split[2], gProc, thresholds[11], MAX_VAL , THRESH_BINARY_INV);
        gpu::threshold(gBGR_split[2], gProc2, thresholds[12], MAX_VAL , THRESH_BINARY);
        gpu::bitwise_and(gProc, gProc2, gProc);
        gpu::bitwise_and(gOut, gProc, gOut);
        
        return gOut;
    }
    
};

int main(int argc, char* argv[])
{

    ros::init(argc, argv, ("image_combinber"));
    ros::NodeHandle nh("~");
    string name, input;
    bool images, maker;
    nh.param<string>("threshold",name,"maker");
    nh.param<bool>("images",images,false);
    nh.param<bool>("maker",maker,(name=="maker"));
    nh.param<string>("input",input,"usb_cam/image_raw");
    int thresh[13] = DEFAULT_THRESHOLDS;

    if(name!="maker")
    {
        string line;
        string file_path = THRESHOLD_PATH+name;
        ifstream file;
        file.open (file_path.c_str());
        if(file.is_open()){
            getline (file,line);
            cout << "Running thresholder with custom thresholds: "<< endl << line << endl;
            istringstream iss(line);
            for(int i = 0; i<13; i++){
                string token;
                getline (iss, token, ' ');
                thresh[i] = atoi(token.c_str());
            }
        }
        else{
            cout << "Unable to open threshold file" << endl;
            return 0;
        }
        file.close();
    }
    else cout << "Running threshold maker" << endl;
    Thresholder t(images,maker,thresh, name, input);
    ros::spin();

    return 0;
}

