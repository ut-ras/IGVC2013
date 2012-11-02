#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <sys/wait.h> 
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <list>
#include <iostream>
#include <sstream>
#include <memory>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
using namespace std;

ros::Publisher vel_pub;
image_transport::Subscriber img_sub;
sensor_msgs::Image cur_img;
geometry_msgs::Twist cur_odom;
ros::Rate loop_rate(5);

void error(char *msg);
void substring(const char* text, int start, int stop, char *new_string);
string char_to_string(char *input_p);
void doserverstuff(int sock, geometry_msgs::Twist *msg);                 
void imageCallback(const sensor_msgs::ImageConstPtr& msg);



