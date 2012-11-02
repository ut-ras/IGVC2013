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
#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
using namespace std;

void error(char *msg);

void substring(const char* text, int start, int stop, char *new_string);

string char_to_string(char *input_p);

int doserverstuff(int sock);
char* getMyIP(int myIp1, int myIp2, int myIp3, int myIp4); 

list<string> get_neighbors();

void odomCallback(const nav_msgs::OdometryConstPtr& msg);
