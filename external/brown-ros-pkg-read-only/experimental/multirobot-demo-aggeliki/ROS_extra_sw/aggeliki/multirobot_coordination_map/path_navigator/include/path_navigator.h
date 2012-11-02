#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <path_navigator/Waypoints.h>
#include <path_navigator/getNextWaypoint.h>
#include <path_navigator/setWaypoints.h>
#include <map_loader/Node.h>

using namespace std;

ros::NodeHandle *n1;
ros::Publisher waypo_pub;
ros::Rate loop_rate(1);

vector<map_loader::Node> waypoints;
int cur_waypoint;

bool setWaypoints(path_navigator::setWaypoints::Request &req, path_navigator::setWaypoints::Response &res);
bool getNextWaypoint(path_navigator::getNextWaypoint::Request &req, path_navigator::getNextWaypoint::Response &res);
void setNextWayp();

void error(char *msg);






