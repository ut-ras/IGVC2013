#include "teleop_server.h"

int main(int argc, char *argv[])
{
     //setup the ROS node
     ros::init(argc, argv, "teleop_server");
     ros::NodeHandle n2;
     vel_pub = n2.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
         
     ros::NodeHandle nh;
     image_transport::ImageTransport it(nh);
     img_sub = it.subscribe("probe/image", 1, imageCallback);       
                 
     //setup the TCP sockets for inter-robot communication
     //main follower functionality
     int sockfd, newsockfd, portno, pid;
     socklen_t clilen;
     struct sockaddr_in serv_addr, cli_addr;
     //sockfd = socket(PF_INET, SOCK_RAW, IPPROTO_TCP);
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = 60050;
     serv_addr.sin_family = AF_INET;
     //serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_addr.s_addr = inet_addr("192.168.0.3");
     //inet_aton("192.168.0.3", &serv_addr.sin_addr);
     serv_addr.sin_port = htons(portno);
    
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR on binding");
     listen(sockfd,5);
     clilen = sizeof(cli_addr);

     while(1)
     {
            int n;
     
            //(1) get velocity message from the teleop netbook
	     newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	     if (newsockfd < 0) 
	          error("ERROR on accept");
	          
             //geometry_msgs::Twist *msg = new geometry_msgs::Twist;
             //doserverstuff(newsockfd, msg);
                 
            //delete msg; //the velocity was published, free up the var
      
            //(3) read the current image and serialize it
            loop_rate.sleep();
            ros::spinOnce();
            loop_rate.sleep();
            uint32_t len = cur_img.serializationLength();
            uint8_t *write_ptr = (uint8_t *)malloc(len*sizeof(uint8_t));
            cur_img.serialize(write_ptr, 1);    

            //send data in chunks
            int cnt = 0;
            while(cnt < 57641)
            {
                int wrsize = BUFSIZ;
                if(57641 - cnt < BUFSIZ)
                    wrsize = 57641 - cnt;
                n = write(newsockfd, (char *)(write_ptr + cnt), wrsize);
                if(n < 0)
                    error("ERROR writing to socket");
                cnt = cnt + wrsize;
            }
            
      }//end while(1)
      
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cur_img = *msg;
}

void doserverstuff(int sock, geometry_msgs::Twist *msg)
{
     int n;
     char buffer[256];
     float lx, az;

     bzero(buffer,256);
     n = read(sock,buffer,255);      
     sscanf(buffer, "lx%faz%f", &lx, &az);
     
     msg->linear.x = lx;
     msg->linear.y = 0;
     msg->linear.z = 0;
     msg->angular.x = 0;
     msg->angular.y = 0;
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
