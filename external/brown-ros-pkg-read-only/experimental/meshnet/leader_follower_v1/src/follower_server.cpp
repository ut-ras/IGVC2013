#include "follower_server.h"

int main(int argc, char *argv[])
{
     //setup the ROS node
     ros::init(argc, argv, "follower_server");
     ros::NodeHandle n;
     //tag_sub = n.subscribe("/tags", 100, tagsCallback);
     tag_sub = n.subscribe("/tags", 1, tagsCallback);
     ros::NodeHandle n2;
     vel_pub = n2.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
     ros::NodeHandle n3;
     //odom_sub = n.subscribe("/odom", 100, odomCallback);
     odom_sub = n.subscribe("/odom", 1, odomCallback);
                 
     //setup the TCP sockets for inter-robot communication
     int sockfd, newsockfd, portno, pid;
     socklen_t clilen;
     struct sockaddr_in serv_addr, cli_addr;
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = 60050;
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     //serv_addr.sin_addr.s_addr = inet_addr("192.168.0.3");
     //inet_aton("192.168.0.3", &serv_addr.sin_addr);
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR on binding");
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
    
     //main follower functionality
     while(1)
     {
             //(1) get velocity message from the leader
	   /*  newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	     if (newsockfd < 0) 
	          error("ERROR on accept");

             geometry_msgs::Twist *msg = new geometry_msgs::Twist;
             doserverstuff(newsockfd, msg);
             
             printf("lx %4.7f ly %4.7f lz %4.7f ax %4.7f ay %4.7f az %4.7f", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
           */ 

            //(2) set the velocity of the icreate using ROS       
          /*  loop_rate.sleep();
            vel_pub.publish(*msg);
            loop_rate.sleep();
            vel_pub.publish(*msg);
            loop_rate.sleep();
            vel_pub.publish(*msg);
            loop_rate.sleep();    
      
            delete msg; //the velocity was published, free up the var
           */      

            //(3) get tag info 
            loop_rate.sleep();
            ros::spinOnce();
            loop_rate.sleep();
                          
 
      }//end while(1)
      
}

void tagsCallback(const ar_alpha::TagsConstPtr& msg)
{
    cout << "(header): seq = " << msg->header.seq << ", timestamp = " << msg->header.stamp << endl;
    
    if(msg->tag_count == 0)
    {
         //do nothing
         cout << "no tags around" << endl;
         geometry_msgs::Twist msgnew;
         msgnew.linear.x = 0;
         msgnew.linear.y = 0;
         msgnew.linear.z = 0;
         msgnew.angular.x = 0;
         msgnew.angular.y = 0;
         msgnew.angular.z = 0.1;   

         loop_rate.sleep();
         vel_pub.publish(msgnew);
         loop_rate.sleep();
         
    }
    else
    { 
	 cout << "number of tags = " << msg->tag_count << endl;
	 			
         //display information about the tags
         for(int i = 0; i < msg->tag_count; i++)
         {
              cout << "tag " << i << ":" << endl;
              cout << "id" << msg->tags[i].id << endl;
              cout << "xRot" << msg->tags[i].xRot << endl;
              cout << "yRot" << msg->tags[i].yRot << endl;
              cout << "zRot" << msg->tags[i].zRot << endl;                            	
         
              //adjust your orientation based on the tag with the greater confidence (current tag for now)
              geometry_msgs::Twist msgnew;
              msgnew.linear.x = 0; //cur_odom.linear.x;
              msgnew.linear.y = 0; //cur_odom.linear.y;
              msgnew.linear.z = 0; //cur_odom.linear.z;
              msgnew.angular.x = 0; //cur_odom.angular.x;
              msgnew.angular.y = 0; //cur_odom.angular.y;
              msgnew.angular.z = msg->tags[i].xRot;//*(2.0/5.0); 
              
              cout << msgnew.linear.x <<endl;
              cout << msgnew.linear.y <<endl;
              cout << msgnew.linear.z <<endl;
              cout << msgnew.angular.x <<endl;
              cout << msgnew.angular.y <<endl;
              cout << msgnew.angular.z <<endl;
             
              loop_rate.sleep();
              vel_pub.publish(msgnew);
              loop_rate.sleep();
         }

    }

    //last10tags.push_front();
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{

    cout << "ODOM CALLBACK" << endl;
    cur_odom.linear.x = msg->twist.twist.linear.x;
    cur_odom.linear.y = msg->twist.twist.linear.y;
    cur_odom.linear.z = msg->twist.twist.linear.z;
    cur_odom.angular.x = msg->twist.twist.angular.x;
    cur_odom.angular.y = msg->twist.twist.angular.y;
    cur_odom.angular.z = msg->twist.twist.angular.z;
}

void doserverstuff(int sock, geometry_msgs::Twist *msg)
{
     int n;
     char buffer[256];
     float lx, ly, lz, ax, ay, az;

     bzero(buffer,256);
     n = read(sock,buffer,255); 
     printf("RECEIVED PACKET: %s\n", buffer);
     
     sscanf(buffer, "lx%fly%flz%fax%fay%faz%f", &lx, &ly, &lz, &ax, &ay, &az);
     
     msg->linear.x = lx;
     msg->linear.y = ly;
     msg->linear.z = lz;
     msg->angular.x = ax;
     msg->angular.y = ay;
     msg->angular.z = az;     
}

void substring(const char* text, int start, int stop, char *new_string)
{
    sprintf(new_string, "%.*s", stop - start, &text[start]);
}

string char_to_string(char *input_p)
{
    string str(input_p);
    return str;
}

void error(char *msg)
{
    perror(msg);
    exit(1);
}
