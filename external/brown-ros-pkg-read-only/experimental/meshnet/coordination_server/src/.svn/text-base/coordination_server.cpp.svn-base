#include "coordination_server.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "coordination_server");
     ros::NodeHandle n1, n2;
     pos_pub = n1.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
     
     handleRequests();
	 
}

void handleRequests()
{
     //receive message from client
     int sockfd, newsockfd, portno, clilen;
     char buffer[BUFSIZ];
     struct sockaddr_in serv_addr, cli_addr;
     int n;

     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = 50050;
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
          error("ERROR on binding");
     listen(sockfd,5); //can accomodate up to 5 requests at the same time
     clilen = sizeof(cli_addr);
     
	 while(1)
	 {
		 newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t *)&clilen);
		 if (newsockfd < 0) 
		      error("ERROR on accept");
		 bzero(buffer, BUFSIZ);

		 n = read(newsockfd,buffer,BUFSIZ);
		 if (n < 0) error("ERROR reading from socket");

		 //get the IP of the client
		 char *client_ip = inet_ntoa(cli_addr.sin_addr);

		 //check whether the received message is an "ok_to_move" command or a /position message
		 if(strcmp(buffer, "ok_to_move") == 0)
		 {		
			 cout << "received ok_to_move msg" << endl;
		 }
		 else{
			 //deserialize received position message
			 boost::shared_ptr<position_tracker::Position> newpos(new position_tracker::Position());            
		     newpos->deserialize((uint8_t *)buffer);

			 cout << "received position = (" << newpos->x << "," << newpos->y << "," << newpos->theta << ") ";
			 printf("from %s\n", client_ip);
		 }

		 close(newsockfd);
	 }

     close(sockfd);

}

void error(char *msg)
{
    perror(msg);
    exit(0);
}
