#include <sys/types.h> 
#include <sys/socket.h>
#include <sys/wait.h> 
#include <netinet/in.h>
#include <netdb.h> 
#include <arpa/inet.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
using namespace std;

ros::Rate loop_rate(5);
double speed;
double turn;
static struct termios oldt;

void disable_waiting_for_enter();
void restore_terminal_settings();
void key_to_motor_cmd(int key, char cmd_str[]);
void error(char *msg);

