#include "path_navigator.h"

//path_navigator arbitrates the navigation: sets the waypoints to be followed, determines the next goal (node) to navigate to
int main(int argc, char *argv[])
{
     ros::init(argc, argv, "path_navigator");
	 n1 = new ros::NodeHandle();
     waypo_pub = n1->advertise<path_navigator::Waypoints>("/waypoints", 1);

     ros::ServiceServer service1 = n1->advertiseService("set_waypoints", setWaypoints);
     ros::ServiceServer service2 = n1->advertiseService("get_next_waypoint", getNextWaypoint);
     ros::spin();     
}

bool setWaypoints(path_navigator::setWaypoints::Request &req, path_navigator::setWaypoints::Response &res)
{
	cout << "inside setWaypoints" << endl;
	waypo_pub.publish(req.w);
	ros::spinOnce();
	cout << "after publishing the waypoints" << endl;

	waypoints = req.w.waypoints;
	cur_waypoint = 0;
	setNextWayp();
	return true;
}

bool getNextWaypoint(path_navigator::getNextWaypoint::Request &req, path_navigator::getNextWaypoint::Response &res)
{
	cout << "cur_waypoint = " << cur_waypoint << endl;
	cout << "waypoints.size() = " << waypoints.size() << endl;
	if(cur_waypoint == waypoints.size())
	{
		n1->setParam("path_navigator/waypoints_reached", true);		
	}
	else
	{
		setNextWayp();
	}
    return true;
}

void setNextWayp()
{
	n1->setParam("step_navigator/goal_pos_x", waypoints[cur_waypoint].p.x);
	n1->setParam("step_navigator/goal_pos_y", waypoints[cur_waypoint].p.y);
	cout << "inside setNextWayp" << endl;
	cur_waypoint++;
}

