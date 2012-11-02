#include "simple_biconnectivity_controller.h"

void msgPassCallback(std::string ip, const position_tracker::PositionConstPtr& msg)
{
	ROS_INFO("Received position [%4.3f, %4.3f] from %s", msg->x, msg->y, ip.c_str());

	//erase previous entry of neighbor's position and create a new one
 	nn_pos.erase(ip);
	nn_pos.insert(pair<string, boost::shared_ptr<position_tracker::Position> >(ip, msg));				 
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

	// Update your subscribers 
	char topic[128];
	ros::Subscriber sub = n->subscribe<position_tracker::Position>(WiFiComm::concatTopicAndIp(topic, "/position", ip), 1, boost::bind(msgPassCallback, std::string(ip), _1));
	subs.push_back(sub); //aggeliki comment: whenever a robot joins the network of the current node (its neighborhood) add a subscriber*/
}


void biconnectivityController()
{
	int neighbors_num = myComm->neighboursList.size();

	//if I have 2 neighbors, be in the middle of them and in vertical distance dvec such that the distance from each neighbor is not bigger than dmax
	//...
	//... -> check the number of neighbors in my local subscriber (maybe that's include in the wifi_comm)
	//...


	//if I have only 1 neighbor, come closer to him and I should be able to sense a 2nd one
	//...
	//...
	//...

				 list< boost::shared_ptr<position_tracker::Position> > nn_pos_list;
				 vector<batman_mesh_info::WifiNN>::iterator it;
				 for(it = wn.neighbors.begin(); it != wn.neighbors.end(); ++it) 
				 {
					 /*char *tmp_ip = new char[strlen((*it).ip.c_str())];
					 strcpy(tmp_ip, (*it).ip.c_str());
					 cout << "(*it).ip = " << (*it).ip << endl;
					 printf("tmp_ip = %s\n", tmp_ip);*/

					 mapType::iterator iter1;
					 for(iter1 = nn_pos.begin(); iter1 != nn_pos.end(); ++iter1)
					 {
						 cout << "iter1->first = " << iter1->first << endl;
						 cout << "iter1->second->x = " << iter1->second->x << endl;
					 }

					 mapType::iterator iter = nn_pos.find((*it).ip);
					 if(iter != nn_pos.end())
					 {
						 nn_pos_list.push_back(iter->second);
						 cout << "inserting neighbors..." << endl;
					 }


}


// *****************************************************************************
// Main function for the multiRobotComm example node
int main( int argc, char** argv )
{
	ros::init(argc, argv, "simple_biconnectivity_controller");
    ros::NodeHandle n1, n2;
    //ros::Subscriber pos_sub = n1.subscribe("/position", 1, positionCallback);
	ros::Publisher pospub = n2.advertise<position_tracker::Position>("/position", 1);

	//JUST TESTING...
	position_tracker::Position *tmp_pos = new position_tracker::Position();
	tmp_pos->x = 2;
	tmp_pos->y = 3;
	cur_pos = *tmp_pos;	

	pospub.publish(cur_pos);
	ros::spinOnce();

	myComm = new WiFiComm(robotJoinedNetwork);
		
	ros::Rate r(0.5);
	while(ros::ok())
	{
   		pospub.publish(cur_pos);
		biconnectivityController();
		ros::spinOnce();
		//r.sleep();	
	}

	delete myComm;

	return 0;
}

// EOF

