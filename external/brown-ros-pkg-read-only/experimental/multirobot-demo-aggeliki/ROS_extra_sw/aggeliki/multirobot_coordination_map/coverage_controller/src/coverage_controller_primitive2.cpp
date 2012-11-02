#include "coverage_controller.h"

void msgPassCallback(std::string ip, const position_tracker::PositionConstPtr& msg)
{
	cout << "Inside msgPassCallback" << endl;

	ROS_INFO("Received position [%4.3f, %4.3f] from %s", msg->x, msg->y, ip.c_str());

	//erase previous entry of neighbor's position and create a new one
 	nn_pos.erase(ip);
	nn_pos.insert(pair<string, position_tracker::Position>(ip, *msg));				 
}

void positionCallback(const position_tracker::PositionConstPtr& msg)
{
	cur_pos = *msg;
}

void robotJoinedNetwork(char * ip)
{
	// Send
	//*** aggeliki: append my ip in the topic to be sent to the neighbors
	myComm->openForeignRelay(ip, my_ip, "/position", true);


	cout << "ip = " << ip << endl;

	// Update your subscribers 
	char topic[128];
	ros::Subscriber sub = n->subscribe<position_tracker::Position>(WiFiComm::concatTopicAndIp(topic, "/position", ip), 1, boost::bind(msgPassCallback, std::string(ip), _1));
	subs.push_back(sub); //aggeliki comment: whenever a robot joins the network of the current node (its neighborhood) add a subscriber*/
}


// *****************************************************************************
// Main function for the multiRobotComm example node
int main( int argc, char** argv )
{
	ros::init(argc, argv, "coverage_controller");
    ros::NodeHandle n1, n2;
	n = new ros::NodeHandle();
    ros::Subscriber pos_sub = n1.subscribe("/position", 1, positionCallback);
	//ros::Publisher pospub = n2.advertise<position_tracker::Position>("/position", 1);

	//JUST TESTING...
	/*position_tracker::Position *tmp_pos = new position_tracker::Position();
	tmp_pos->x = 2;
	tmp_pos->y = 3;
	cur_pos = *tmp_pos;	

	pospub.publish(cur_pos);
	ros::spinOnce(); */

	myComm = new WiFiComm(robotJoinedNetwork);
		
	ros::Rate r(0.5);
	while(ros::ok())
	{
   		//pospub.publish(cur_pos);
		//nextMoveController();
		ros::spinOnce();
		//r.sleep();	
	}

	delete myComm;

	return 0;
}


void nextMoveController()
{
	//Get your current position and the position of your neighbors and find their closest nodes in the graph map. Determine which neighboring nodes are in addition available to move to (hopefully the nearby robot will not decide to move to the same location that is now available)
   	 ros::NodeHandle n1, n2;
  	 ros::ServiceClient client = n1.serviceClient<path_generator::availableNextHops>("available_next_hops");
      path_generator::availableNextHops srv; 
	 
	 vector<position_tracker::Position> neighbor_pos;
	 vector<wifi_comm::WiFiNeighbour>::iterator it;
	 for(it = myComm->neighboursList.neighbours.begin(); it != myComm->neighboursList.neighbours.end(); ++it) 
	 {
		mapType::iterator iter = nn_pos.find((*it).ip);
		if(iter != nn_pos.end())
		{
			neighbor_pos.push_back(iter->second);
			cout << "inserting neighbors..." << endl;
		}
	 }

	 srv.request.cur_pos = cur_pos;
	 srv.request.neighbor_pos = neighbor_pos;

     if(!client.call(srv))
     {
         cout << "Failed to call service available_next_hops" << endl;
     }

	vector<position_tracker::Position> avail_next_hops = srv.response.avail_next_hops; 
	vector<position_tracker::Position>::iterator it2;
	 for(it2 = avail_next_hops.begin(); it2!=avail_next_hops.end(); ++it2)
	 {
		 cout << "available hop: x = " << it2->x << ", y = " << it2->y << endl;
	 }

	//calculate your score for staying in the current position and the score for moving to each of the available next nodes
	double my_penalty = 0;
	vector<double> penalties; //one entry for each of the avail_neigh_nodes
	double dmax = 6; //optimal distance between 2 robots (in m)
	cout << "avail_next_hops.size() = " << avail_next_hops.size() << endl;
	cout << "neighbor_pos.size() = " << neighbor_pos.size() << endl;		
	for(it2 = avail_next_hops.begin(); it2 != avail_next_hops.end(); ++it2)
	{
		position_tracker::Position p = *it2;

		double local_penalty = 0;
		vector<position_tracker::Position>::iterator it3;
		for(it3 = neighbor_pos.begin(); it3 != neighbor_pos.end(); ++it3)
		{
			cout << "p = (" << p.x << ", " << p.y << ")" << endl;
			double dist = sqrt(pow(it3->x - p.x, 2) + pow(it3->y - p.y, 2));
			cout << "dist = " << dist << endl;
			double tmp_penalty = 0;
			if(dist < dmax)
				tmp_penalty = dmax - dist;
			else if(dist >= dmax)
				tmp_penalty = 2*(dist-dmax);
			cout << "tmp_penalty = " << tmp_penalty << endl;

			local_penalty = local_penalty + tmp_penalty;
		}
		penalties.push_back(local_penalty);			
	}

	vector<position_tracker::Position>::iterator it3;
	for(it3 = neighbor_pos.begin(); it3 != neighbor_pos.end(); ++it3)
	{
		double dist = sqrt(pow(it3->x - cur_pos.x, 2) + pow(it3->y - cur_pos.y, 2));
		cout << "my dist = " << dist << endl;
		double tmp_penalty = 0;
		if(dist < dmax)
			tmp_penalty = dmax - dist;
		else if(dist >= dmax)
			tmp_penalty = 2*(dist-dmax);
		cout << "my tmp_penalty = " << tmp_penalty << endl;

		my_penalty = my_penalty + tmp_penalty;
	}

	//transition to the node that reduces the penalty function the most
	vector<double>::iterator it4;
	int cnt = 0;
	double max_diff = 0;
	position_tracker::Position max_p = cur_pos;
	cout << "my_penalty = " << my_penalty << endl;
	for(it4 = penalties.begin(); it4 != penalties.end(); ++it4)
	{
		double diff = (*it4) - my_penalty;
		cout << "(*it4) penalty = " << (*it4) << endl;
		cout << "diff = " << diff << endl;
		if(diff < 0 && abs(diff) > max_diff)
		{
			max_diff = abs(diff);
			max_p = avail_next_hops.at(cnt);
		}
		cnt++;
	}
	
	cout << "max_p = (" << max_p.x << ", " << max_p.y << ")" << endl;

	//publish the next goal position (through the path navigator!)
	ros::ServiceClient client2 = n2.serviceClient<path_navigator::setWaypoints>("set_waypoints");
     path_navigator::setWaypoints srv2; 
	path_navigator::Waypoints *w = new path_navigator::Waypoints();
	map_loader::Node *mnode = new map_loader::Node();
	mnode->p = max_p;
	w->waypoints.push_back(*mnode);           
     srv2.request.w = *w;

     if(!client2.call(srv2))
     {
         cout << "Failed to call service set_waypoints" << endl;
     }

	//TEST!!! with 2, 2, 3, 5 robots

}
