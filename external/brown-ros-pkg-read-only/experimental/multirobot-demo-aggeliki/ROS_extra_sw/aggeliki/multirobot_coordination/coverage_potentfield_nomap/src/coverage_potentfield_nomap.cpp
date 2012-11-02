#include "coverage_potentfield_nomap.h"

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
	cout << "pushing" << endl;
	subs.push_back(sub); //aggeliki comment: whenever a robot joins the network of the current node (its neighborhood) add a subscriber*/
	cout << "pushed" << endl;
}


// *****************************************************************************
// Main function for the multiRobotComm example node
int main( int argc, char** argv )
{
	ros::init(argc, argv, "coverage_potentfield_nomap");
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
	double Ds = 0.3;    //move for Ds in the desired direction
	//int T = 80;     //wireless signal threshold for acceptable connections (when using lots of robots)
	unsigned int T = 90;     //wireless signal threshold for acceptable connections (when using few robots)


	//get the positions of your neighbors
	vector<position_tracker::Position> neighbor_pos;
	vector<wifi_comm::WiFiNeighbour>::iterator it;
	for(it = myComm->neighboursList.neighbours.begin(); it != myComm->neighboursList.neighbours.end(); ++it)
	{
	    mapType::iterator iter = nn_pos.find((*it).ip);
	    cout << "neighbor " << (*it).ip << ", quality = " << (*it).quality << endl;
	    cout << "iter != nn_pos.end() " << (iter != nn_pos.end()) << endl;
	    cout << "(*it).quality > T " << ((*it).quality > T) << endl;
	    if(iter != nn_pos.end() && (*it).quality > T)
	    {
			neighbor_pos.push_back(iter->second);
			cout << "inserting neighbor " << (*it).ip << "..." << endl;
	    }
	}

	//calculate the potentials from all your neighbors
	double next_step_x = 0, next_step_y = 0;
	double denom = 0;
	vector<position_tracker::Position>::iterator it1;
	for(it1 = neighbor_pos.begin(); it1 != neighbor_pos.end(); ++it1)
	{
		double v_mag = sqrt(pow(cur_pos.x - it1->x, 2) + pow(cur_pos.y - it1->y, 2));
		double v_x = -(it1->x - cur_pos.x);
		double v_y = -(it1->y - cur_pos.y);

		//take the weighted sum over all neighbors based on the current distance from the current position (v_mag)
		next_step_x = next_step_x + v_x/pow(v_mag, 2);
		next_step_y = next_step_y + v_y/pow(v_mag, 2);
		denom = denom + 1/pow(v_mag, 2);

		cout << "v_mag = " << v_mag << endl;
		cout << "v_x = " << v_x << endl;
		cout << "v_y = " << v_y << endl;
	}

	//finalize the weighted sum over all neighbors
	if (denom > 0) { // has neighbors
	  next_step_x = next_step_x/denom;
	  next_step_y = next_step_y/denom;
	}
	cout << "next_step_x = " << next_step_x << endl;
	cout << "next_step_y = " << next_step_y << endl;

	//normalize resulting direction vector (next_step_x, next_step_y) and take a Ds step towards the desired direction
	double dist = sqrt(pow(next_step_x, 2) + pow(next_step_y, 2));
	double v_new_x = 0, v_new_y = 0;
	if(dist > 0) {
	  v_new_x = (next_step_x/dist)*Ds;
	  v_new_y = (next_step_y/dist)*Ds;
	}

	cout << "v_new_x = " << v_new_x << endl;
	cout << "v_new_y = " << v_new_y << endl;

	//transition to the node that reduces the penalty function the most
	position_tracker::Position max_p;
	max_p.x = cur_pos.x + v_new_x;
	max_p.y = cur_pos.y + v_new_y;

	cout << "cur_pos = (" << cur_pos.x << ", " << cur_pos.y << ")" << endl;
	cout << "max_p = (" << max_p.x << ", " << max_p.y << ")" << endl;
	max_p.theta = cur_pos.theta;

	//publish the next goal position
	ros::NodeHandle n1;
	n1.setParam("step_navigator/goal_pos_x", max_p.x);
	n1.setParam("step_navigator/goal_pos_y", max_p.y);

}
