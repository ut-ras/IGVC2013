#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h>

using namespace std;

ros::NodeHandle *n1;
ros::Rate loop_rate(1);

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "simple_step_navigator_nomap_controller");
	n1 = new ros::NodeHandle();

	while(1)
	{
		n1->setParam("step_navigator/goal_pos_x", 15);
		n1->setParam("step_navigator/goal_pos_y", 13);
		ros::spinOnce();
	}     
}
