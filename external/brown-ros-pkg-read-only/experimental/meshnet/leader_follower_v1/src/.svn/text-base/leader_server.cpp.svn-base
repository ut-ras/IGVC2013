/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include "leader_server.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "leader_server");

     while(1)
     {


     //------------------------------------------
     //set the angular velocity of the icreate using ROS
     ros::NodeHandle n2;
     ros::Publisher vel_pub = n2.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
     ros::Rate loop_rate(5); //how fast the messages are being sent
     std::stringstream ss;
     //ss << "Hello there! This is message [" << count << "]";
     //while(ros::ok())
     //{
     geometry_msgs::Twist msg;
     //msg.linear.x = 1;
     msg.linear.x = 0.25;
     msg.linear.y = 0;
     msg.linear.z = 0;
     msg.angular.x = 0;
     msg.angular.y = 0;
     msg.angular.z = 1;

     /*vel_pub.publish(msg);
     loop_rate.sleep();
     vel_pub.publish(msg);
     loop_rate.sleep();
     vel_pub.publish(msg);
     loop_rate.sleep();
     vel_pub.publish(msg);
     loop_rate.sleep();
     vel_pub.publish(msg);
     loop_rate.sleep();*/
     
     
     
     //------------------------------------------


     int sockfd, newsockfd, portno, pid;
     socklen_t clilen;
     struct sockaddr_in serv_addr, cli_addr;
     /*if (argc < 2) 
     {
         fprintf(stderr,"ERROR, no port provided\n");
         exit(1);
     }*/
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     //portno = atoi(argv[1]);
     portno = 60050;
     serv_addr.sin_family = AF_INET;
     //serv_addr.sin_addr.s_addr = INADDR_ANY;
     //serv_addr.sin_addr.s_addr = inet_addr("192.168.0.3");
     inet_aton("192.168.0.3", &serv_addr.sin_addr);
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR on binding");
     listen(sockfd,5);
     clilen = sizeof(cli_addr);

 	     char str[1024];
 
	     newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	     if (newsockfd < 0) 
	          error("ERROR on accept");

             int res = doserverstuff(newsockfd);
              
              

         int status, died;
             //get 											 from the iCreate
              ros::NodeHandle n;
              ros::Rate loop_rate(5); //how fast the messages are being received
              ros::Subscriber odom_sub = n.subscribe("/odom", 100, odomCallback);
              loop_rate.sleep();
              ros::spinOnce();
              loop_rate.sleep();
              
              //sleep(3);
              //get the list with the neighbors
              list<string> nbgd = get_neighbors();
              
              //display neighbors
              list<string>::iterator i;
              int cnt = 1;
              for(i = nbgd.begin(); i != nbgd.end(); ++i)
              {
                  cout << "NEIGHBOR " << *i << endl;
                  
                  //send a hello to the neighbor
                  int sockfd2, n2;
	    	  struct sockaddr_in serv_addr2;
	          struct hostent *server2;

	          char *buffer2;
	          sockfd2 = socket(AF_INET, SOCK_STREAM, 0);
	          if (sockfd2 < 0) 
		      error("ERROR opening socket");

	          bzero((char *) &serv_addr2, sizeof(serv_addr2));
	          serv_addr2.sin_family = AF_INET;
	          //serv_addr2.sin_addr.s_addr = inet_addr("192.168.0.2");
	          string tmp_str = *i;
	          char *tmp_addr = new char[tmp_str.size() + 1];
	          strcpy(tmp_addr, tmp_str.c_str());
	          
                  inet_aton(tmp_addr, &serv_addr2.sin_addr);
	          serv_addr2.sin_port = htons(60050);
	         if (connect(sockfd2,(struct sockaddr *) &serv_addr2,sizeof(serv_addr2)) < 0) 
		      error("ERROR connecting");
	         buffer2 = "hello";
	         n2 = write(sockfd2,buffer2,strlen(buffer2));
	         if (n2 < 0) 
		     error("ERROR writing to socket");

                  cnt++;
              }
              
              //send the odometry to the neighbors
              //(for now: just send a hello message)
              //...
              
          }

	  died = wait(&status); //NO, the parent shouldn't wait for the child to finish => WHAT ABOUT GHOST NODES THOUGH???
     
      }

      }//end while(1)
      
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    ROS_INFO("Odometry results:");
    ROS_INFO("Seq: %d", msg->header.seq);

    ROS_INFO("Pose: (%4.7f, %4.7f, %4.7f)", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    ROS_INFO("Orientation: (%4.7f, %4.7f, %4.7f, %4.7f)", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    //==> OR twist is probably the velocity (linear, angular): like cmd_vel
    //Attributes I haven't displayed (NOTE: no difference between orientation and twist -> basically I can ignore the rest):
    //msg.pose.covariance
    //msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z 
    //msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z 
    //msg.twist.covariance
}
void substring(const char* text, int start, int stop, char *new_string)
{
    sprintf(new_string, "%.*s", stop - start, &text[start]);
}

string char_to_string(char *input_p)
{
    //shared_ptr<char> p(input_p, &free);
    //sring str(p.get());
    //return str;
	
    string str(input_p);
    //free(input_p);
    return str;
}

void error(char *msg)
{
    perror(msg);
    exit(1);
}

int doserverstuff(int sock)
{
     int n;
     char buffer[256];

     bzero(buffer,256);
     n = read(sock,buffer,255); 
     printf("RECEIVED PACKET: %s\n", buffer);
     
     /*--- CAN also read socket contents using FILE descriptors (will need fdopen, read, fgets, flush) ---*/

/*//=> BEGINNING OF LAST COMMENTS
     //parse the input message
     int data1, data2, dst_ip1, dst_ip2, dst_ip3, dst_ip4;
     sscanf(buffer, "dst %d.%d.%d.%d data %d,%d", &dst_ip1, &dst_ip2, &dst_ip3, &dst_ip4, &data1, &data2);
     printf("RECEIVED PACKET: %s\n", buffer);
     printf("dst IP: %d.%d.%d.%d\n", dst_ip1, dst_ip2, dst_ip3, dst_ip4);
     printf("DATA: \n%d\n%d\n", data1, data2);

     if (n < 0) error("ERROR reading from socket");

     for(int i = 1; i < 200; i++)
         //send an ACK to the sender
         n = write(sock,"I got your message",18);

     if (n < 0) error("ERROR writing to socket");
     //return 0; 

     //get your IP address
     int myIp1, myIp2, myIp3, myIp4;
     char* buf2;
     buf2 = getMyIP(myIp1, myIp2, myIp3, myIp4); 
     printf("result from reading my IP: START%sEND\n", buf2);
     sscanf(buf2, "%d.%d.%d.%d\n", &myIp1, &myIp2, &myIp3, &myIp4);
     
     printf("my IP: %d.%d.%d.%d\n", myIp1, myIp2, myIp3, myIp4);

     //compare your current IP with the IP of the packet destination
     if(dst_ip1 == myIp1 && dst_ip2 == myIp2 && dst_ip3 == myIp3 && dst_ip4 == myIp4) //if same
     {
         //do stuff with the data, e.g. display the data
         printf("The data received is: %d, %d\n", data1, data2);
     }
     else //forward the packet
     {
         int nxt_ip1, nxt_ip2, nxt_ip3, nxt_ip4;

         //read neighborhood info from BATMAN and identify "next hop" node
         get_next_hop(dst_ip1, dst_ip2, dst_ip3, dst_ip4, nxt_ip1, nxt_ip2, nxt_ip3, nxt_ip4);
 
         nxt_ip1 = 2;

         //send the packet to "next hop" node
         if(nxt_ip1 == -1)
         {
             error("DESTINATION NODE unreachable");
         }
         else
         {
              //send the packet to "next hop"
              //...
              //...
              //...
             

 /*             //basically code copied from .../init_code/client.c
	    int sockfd2, n2;
	    struct sockaddr_in serv_addr2;
	    struct hostent *server2;

	    char buffer2[256];
	    sockfd2 = socket(AF_INET, SOCK_STREAM, 0);
	    if (sockfd2 < 0) 
		error("ERROR opening socket");

	    bzero((char *) &serv_addr2, sizeof(serv_addr2));
	    serv_addr2.sin_family = AF_INET;
	    //serv_addr2.sin_addr.s_addr = inet_addr("192.168.0.2");
            inet_aton("192.168.0.2", &serv_addr2.sin_addr);
	    serv_addr2.sin_port = htons(60050);
	    if (connect(sockfd2,(struct sockaddr *) &serv_addr2,sizeof(serv_addr2)) < 0) 
		error("ERROR connecting");
	    printf("Please enter the message: ");
	    bzero(buffer2,256);
	    fgets(buffer2,255,stdin);
	    n2 = write(sockfd2,buffer2,strlen(buffer2));
	    if (n2 < 0) 
		 error("ERROR writing to socket");
*/ //=> END OF LAST COMMENTS
    /*     }
     }*/

     return 1;
}

char* getMyIP(int myIp1, int myIp2, int myIp3, int myIp4) 
{
     FILE *ptr;
     //int myIp1, myIp2, myIp3, myIp4;
     char buf[BUFSIZ];
     char *command = "ifconfig wlan0 | grep \"inet addr:\" | cut -d: -f2 | awk '{ print $1}'";

     if((ptr = popen(command, "r")) != NULL)
     {
        fgets(buf, BUFSIZ, ptr);
     }
     pclose(ptr);

     return buf;
     //printf("my IP here: %d.%d.%d.%d\n", *myIp1, *myIp2, *myIp3, *myIp4);
}

//RENAME to get_neighbors (returns a list<string>)
list<string> get_neighbors()
{
     FILE *ptr;
     char buf[BUFSIZ];
     //MAKE SURE that when you fire up batman you also set the originator interval: e.g. sudo batmand -o 300 wlan0
     char *command = "sudo batmand -b -c -d 1";
     list<string> neigbList;     

     /*NO NEED to check if there are batman nodes in range (if no batman nodes in range, server is still waiting for clients to connect)*/

     //FOR NOW, just find neighbors of current node and return them as a list of strings
     if((ptr = popen(command, "r")) != NULL)
     {
        //read and ignore the first line
        fgets(buf, BUFSIZ, ptr);

	while(fgets(buf, BUFSIZ, ptr) != NULL)
	{
	    string str, ip_str;
            string::size_type pos1, pos2;

            //get the IP of current line's next hop node
	    str = char_to_string(buf);
            pos1 = str.find_first_of(')');
            pos2 = str.find_first_of('[');
	    ip_str = str.substr(pos1 + 6, pos2 - pos1 - 7);
            //neigbList.push_back(ip_str);            
            neigbList.push_front(ip_str);

            cout << str << endl;                                   
            cout << "IP: " << ip_str << endl;             
	} 
     }
     pclose(ptr);

     //the neighbors of the current node are the unique next hop nodes
     neigbList.unique();
     return neigbList;

}
