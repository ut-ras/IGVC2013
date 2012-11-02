#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <signal.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <wifi_comm/WiFiNeighboursList.h>
#include <position_tracker/Position.h>
#include <path_generator/availableNextHops.h>
#include <path_navigator/Waypoints.h>
#include <path_navigator/setWaypoints.h>
#include <map_loader/Node.h>
#include "std_msgs/String.h"

#include <wifi_comm/wifi_comm_lib.h>

using namespace std;
typedef map<string, position_tracker::Position> mapType;

WiFiComm * myComm;
ros::NodeHandle * n;

//*** aggeliki: my ip (ideally should be obtained automatically -> or maybe through the wifi_comm::wifi_discovery_node)
char *my_ip = "192.168.0.6";
position_tracker::Position cur_pos;

std::vector<ros::Subscriber> subs;

std::map<string, position_tracker::Position > nn_pos; //the last position message from  neighboring IPs

void nextMoveController();


