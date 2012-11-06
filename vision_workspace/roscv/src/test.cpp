#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	printf("ya");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "roscv");

	ros::NodeHandle nh;
 	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/gscam/image_raw", 1, imageCallback);
     
	ros::spin();
   
    	return 0;
}