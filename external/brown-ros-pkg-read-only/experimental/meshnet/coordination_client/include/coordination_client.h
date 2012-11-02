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
#include <batman_mesh_info/WifiNNs.h>
#include <batman_mesh_info/WifiNN.h>
#include <position_tracker/Position.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
using namespace std;

ros::Subscriber pos_sub;
ros::Subscriber wifi_sub;
ros::Rate loop_rate(1);
position_tracker::Position cur_pos;

void positionCallback(const position_tracker::PositionConstPtr& msg);
void wifiCallback(const batman_mesh_info::WifiNNsConstPtr& msg);
void error(char *msg);

