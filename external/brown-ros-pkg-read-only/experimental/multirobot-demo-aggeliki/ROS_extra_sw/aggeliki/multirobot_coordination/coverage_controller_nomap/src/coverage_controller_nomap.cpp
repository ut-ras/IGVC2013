#include "coverage_controller_nomap.h"

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
	ros::init(argc, argv, "coverage_controller_nomap");
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
//	while(ros::ok())
	while(1)
	{
   		//pospub.publish(cur_pos);
		nextMoveController();
		ros::spinOnce();
		//r.sleep();	
	}

	delete myComm;

	return 0;
}


void nextMoveController()
{
	//consider moving for Ds in each of 8 directions (N, S, E, W, NE, NW, SE, SW)
	double Ds = 0.3;
	vector<position_tracker::Position> avail_next_hops, avail_next_hops_init;
	position_tracker::Position *tmp_pos1 = new position_tracker::Position();
	tmp_pos1->x = cur_pos.x + Ds;
	tmp_pos1->y = cur_pos.y;
	position_tracker::Position *tmp_pos2 = new position_tracker::Position();
	tmp_pos2->x = cur_pos.x - Ds;
	tmp_pos2->y = cur_pos.y;
	position_tracker::Position *tmp_pos3 = new position_tracker::Position();
	tmp_pos3->x = cur_pos.x;
	tmp_pos3->y = cur_pos.y + Ds;
	position_tracker::Position *tmp_pos4 = new position_tracker::Position();
	tmp_pos4->x = cur_pos.x;
	tmp_pos4->y = cur_pos.y - Ds;
	position_tracker::Position *tmp_pos5 = new position_tracker::Position();
	tmp_pos5->x = cur_pos.x + Ds/sqrt(2);
	tmp_pos5->y = cur_pos.y + Ds/sqrt(2);
	position_tracker::Position *tmp_pos6 = new position_tracker::Position();
	tmp_pos6->x = cur_pos.x + Ds/sqrt(2);
	tmp_pos6->y = cur_pos.y - Ds/sqrt(2);
	position_tracker::Position *tmp_pos7 = new position_tracker::Position();
	tmp_pos7->x = cur_pos.x - Ds/sqrt(2);
	tmp_pos7->y = cur_pos.y + Ds/sqrt(2);
	position_tracker::Position *tmp_pos8 = new position_tracker::Position();
	tmp_pos8->x = cur_pos.x - Ds/sqrt(2);
	tmp_pos8->y = cur_pos.y - Ds/sqrt(2);
	avail_next_hops_init.push_back(*tmp_pos1);
	avail_next_hops_init.push_back(*tmp_pos2);
	avail_next_hops_init.push_back(*tmp_pos3);
	avail_next_hops_init.push_back(*tmp_pos4);
	avail_next_hops_init.push_back(*tmp_pos5);
	avail_next_hops_init.push_back(*tmp_pos6);
	avail_next_hops_init.push_back(*tmp_pos7);
	avail_next_hops_init.push_back(*tmp_pos8);

	//get the positions of your neighbors	
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

	//remove all the next moves that bring your "on top of" a neighboring robot
	vector<position_tracker::Position>::iterator it2;
     for(it2 = avail_next_hops_init.begin(); it2!=avail_next_hops_init.end(); ++it2)
	{
	     cout << "available hop: x = " << it2->x << ", y = " << it2->y << endl;
		avail_next_hops.push_back(*it2);
		vector<position_tracker::Position>::iterator it3;
		for(it3 = neighbor_pos.begin(); it3!=neighbor_pos.end(); ++it3)
		{
			double dist = sqrt(pow(it2->x - it3->x, 2) + pow(it2->y - it3->y, 2));
			if(dist < Ds)
			{
				avail_next_hops.pop_back();
				break;
			}
		}
	}

	//calculate your score for staying in the current position and the score for moving to each of the available next nodes
	double my_penalty = 0;
	vector<double> penalties; //one entry for each of the avail_neigh_nodes
	double dmax = 2.5; //optimal distance between 2 robots (in m) -> AIlab corridor: 8, big corridor: 13
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
			//gaussian penalty
			if(dist < dmax)
				tmp_penalty = exp(-dist);
			else if(dist >= dmax)
				tmp_penalty = exp(-(2*dmax - dist));
			cout << "tmp_penalty = " << tmp_penalty << endl;

			local_penalty = local_penalty + tmp_penalty;
			cout << "local_penalty = " << local_penalty << endl;
		}
		penalties.push_back(local_penalty);			
	}

	vector<position_tracker::Position>::iterator it3;
	for(it3 = neighbor_pos.begin(); it3 != neighbor_pos.end(); ++it3)
	{
		double dist = sqrt(pow(it3->x - cur_pos.x, 2) + pow(it3->y - cur_pos.y, 2));
		cout << "my dist = " << dist << endl;
		double tmp_penalty = 0;
		//gaussian penalty
		if(dist < dmax)
			tmp_penalty = exp(-dist);
		else if(dist >= dmax)
			tmp_penalty = exp(-(2*dmax - dist));
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
	max_p.theta = cur_pos.theta;

	//publish the next goal position
	ros::NodeHandle n1;
	n1.setParam("step_navigator/goal_pos_x", max_p.x);
	n1.setParam("step_navigator/goal_pos_y", max_p.y);

}
