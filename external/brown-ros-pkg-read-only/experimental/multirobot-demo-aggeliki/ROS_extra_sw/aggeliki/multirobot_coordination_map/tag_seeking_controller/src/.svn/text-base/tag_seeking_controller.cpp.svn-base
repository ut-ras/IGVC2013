#include "tag_seeking_controller.h"

int main(int argc, char *argv[])
{
     //setup the ROS node
     ros::init(argc, argv, "tag_seeking_controller");
	 n1 = new ros::NodeHandle();
     tag_sub = n1->subscribe("/tags", 1, tagsCallback);     
}

void tagsCallback(const ar_recog::TagsConstPtr& msg)
{
	tags = *msg;

	if(tag_found)
	{	
		ROS_INFO("Mission complete!");
		return;
	}

	for(tag 1 till end)
	{
		if(current tag belongs to the list of tags I want to find) //probably list of tags to find should be a topic
		{
			//then stop
			new Node np;
			np->p = cur_pos;
			new Waypoints wp;
			wp.w.push_back(np);
			path_navigator::setWaypoints srv;
			srv.req.q = wp;
			if(!call(srv))
			{
				ROS_ERROR("Failed to call setWaypoints service!");
			}
			tag_found = true;
		}
	}
		

}

void explore()
{
	n1->setParam("path_navigator/waypoints_reached", waypreached);		

	if(waypreached)
	{
		path_generator.generatenewRandomWalk path;
		path_navigator.setWaypoints(path);
	}
}

