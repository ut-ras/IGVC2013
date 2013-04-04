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
#include <math.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/LaserScan.h>

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

double RATE = 10,
       LOG_SCALE = 20.0,
       PIXELS_PER_METER = 281.0,
       DISTANCE_FROM_FRONT = .31,
       RAY_THICKNESS = 1;

ros::Publisher scan_pub;

void processImage(Mat img) {
    //
    // Scan through the image and calculate distances
    //
    Size img_size = img.size();

    unsigned int num_readings = 1 + img_size.height/RAY_THICKNESS,
                 max_dist = img_size.width;

    double angle_width = M_PI/(double)num_readings;

    double ranges[num_readings];

    uint8_t* pixelPtr = (uint8_t*)img.data;

    for (int i = 0; i < num_readings - 1; i++) {
        unsigned int row = (num_readings - 1 - i)*RAY_THICKNESS,
                     c = 0;

        for (c = 0; c < max_dist; c++) {
            bool flag = false;

            for (unsigned int r = row; r < row + RAY_THICKNESS; r++) {
                if (pixelPtr[r*max_dist + c] > 0) {
                    flag = true;
                    break;
                }
            }

            if (flag) {
                break;
            }
        }

        ranges[i] = exp(c/LOG_SCALE)/PIXELS_PER_METER + DISTANCE_FROM_FRONT;
    }
    
    ranges[0] = ranges[1];

    //
    // populate the LaserScan message
    //
    ros::Time scan_time = ros::Time::now();

    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    scan.angle_increment = angle_width;

    scan.ranges.resize(num_readings);
    for (unsigned int i = 0; i < num_readings; ++i){
        scan.ranges[i] = ranges[i];
    }

    scan_pub.publish(scan);
}

bool imgInit = false;
Mat curImg;

void callback(const sensor_msgs::ImageConstPtr& msg) {
    Mat src;   
 
    try {
        src = cv_bridge::toCvCopy(msg, enc::MONO8)->image;
        curImg = src;
        imgInit = true;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // processsImage(src);

    // imshow("Input from log polar", src);
    // waitKey(30);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "vision_decision");

    namedWindow("Input from log polar");

    ros::NodeHandle nh;

    scan_pub = nh.advertise<sensor_msgs::LaserScan>(
        "/image_scan", 
        1 // outgoing buffer size
    );
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe(
        "/log_polar_transformed", 
        1, // incoming buffer size
        &callback
    );

    ros::Rate loop_rate(RATE);
    while (ros::ok()) {
        if (imgInit) {
            processImage(curImg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
