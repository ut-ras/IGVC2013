#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <position_tracker/Position.h>
#include <position_tracker/SetPosition.h>
#include <irobot_create_2_1/SensorPacket.h>
#include <map_loader/LineMap.h>
#include <map_loader/Line.h>
#include <math.h>
#define PI 3.14159265

using namespace std;

class NodeHandleWrapper
{
	public:
	ros::NodeHandle *local_node_handle;

    public:
	NodeHandleWrapper()
	{
		local_node_handle = new ros::NodeHandle(); 
	}	
};

NodeHandleWrapper *nhw;
vector<map_loader::Line> map4;

ros::Publisher vel_pub;
ros::Subscriber pos_sub;
ros::Subscriber bump_sub;
ros::Subscriber lmap_sub;
ros::Rate loop_rate(1);
position_tracker::Position cur_pos;
irobot_create_2_1::SensorPacket cur_sensors;
int prevBump = 0;
position_tracker::Position latestBumpPos; 
int ok_to_drive = 1;

void lineMapCallback(const map_loader::LineMapConstPtr& msg);
void bumperCallback(const irobot_create_2_1::SensorPacketConstPtr& msg);
position_tracker::Position getClosestLeftLineProjection();
position_tracker::Position getClosestRightLineProjection();
void substring(const char* text, int start, int stop, char *new_string);
string char_to_string(char *input_p);

double det(double A[2][2]);
void inv(double A[2][2], double IA[2][2]);
void solve(double A[2][2], double C[2], double S[2]);
void positionCallback(const position_tracker::PositionConstPtr& msg);
void error(char *msg);




