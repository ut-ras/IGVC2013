#include "coordination_client.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "coordination_client");
     ros::NodeHandle n1, n2;
     pos_sub = n1.subscribe("/position", 1, positionCallback);       
     wifi_sub = n2.subscribe("/wifiNNs", 1, wifiCallback);       

	 while(1)
	 {
     	ros::spinOnce();
	 }
}

void positionCallback(const position_tracker::PositionConstPtr& msg)
{
   cur_pos = *msg;
}

void wifiCallback(const batman_mesh_info::WifiNNsConstPtr& msg)
{
	 //serialize the current position of the robot => DON'T need to check if irobot_create_2_1 and position_tracker are already running 
	 //if(cur_pos.x < 0) 
		//return;

	 cur_pos.x = 1001;
	 cur_pos.y = 1002;
	 cur_pos.theta = 0.1234;

     uint32_t len = cur_pos.serializationLength();
     uint8_t *write_ptr = (uint8_t *)malloc(len*sizeof(uint8_t));
     cur_pos.serialize(write_ptr, 1);

	 cout << "serialization length = " << len << endl;    

	 //read all the neighbors in terms of wifi and send them the serialzed position
	 batman_mesh_info::WifiNNs wn = *msg;	 
     vector<batman_mesh_info::WifiNN>::iterator it;
	 cout << "neighbors list:" << endl;
     for(it = wn.neighbors.begin(); it != wn.neighbors.end(); ++it) 
	 {
		//connect to the server (NEED also to put a time limit for waiting for a connection)		 
		int sockfd, portno, n;
		struct sockaddr_in serv_addr;
		struct hostent *server;
		
		//print the neighbors:
		cout << "NN: " << (*it).ip << endl;
		cout << "signal: " << (*it).quality << endl;

		//const char *serv_ip = ((*it).ip).c_str();  //get the IP of a neighbor
		char *serv_ip = "192.168.0.2";  //for testing: get a prespecified IP

		portno = 50060;
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0) 
		    error("ERROR opening socket");
		server = gethostbyname(serv_ip);
		if (server == NULL) {
		    fprintf(stderr,"ERROR, no such host\n");
		    exit(0);
		}
		bzero((char *) &serv_addr, sizeof(serv_addr));
		serv_addr.sin_family = AF_INET;
		bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
		serv_addr.sin_port = htons(portno);
		if (connect(sockfd,(sockaddr *)&serv_addr,sizeof(serv_addr)) < 0) 
		    error("ERROR connecting");

		//send an "ok_to_move" command		
		/*char *buffer = "ok_to_move";
		n = write(sockfd,buffer,strlen(buffer));*/
	
		//send a /position message	
		n = write(sockfd,write_ptr,len*sizeof(uint8_t));
		if (n < 0) 
		     error("ERROR writing to socket");

		close(sockfd);
	}
}

void error(char *msg)
{
    perror(msg);
    exit(0);
}
