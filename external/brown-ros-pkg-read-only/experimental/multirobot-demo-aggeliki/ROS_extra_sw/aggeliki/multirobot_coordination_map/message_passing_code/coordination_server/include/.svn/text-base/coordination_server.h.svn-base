#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <list>
#include <numeric>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <position_tracker/Position.h>
#include <geometry_msgs/Twist.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <arpa/inet.h>
using namespace std;

ros::Publisher pos_pub;
ros::Rate loop_rate(1);
position_tracker::Position cur_pos;

void positionCallback(const position_tracker::PositionConstPtr& msg);
void handleRequests();
void error(char *msg);

