#include "teleop_client.h"

int main(int argc, char *argv[])
{
     //setup the ROS node
     ros::init(argc, argv, "teleop_client");
     ros::NodeHandle nh;
     cvNamedWindow("view");
     cvStartWindowThread();

     disable_waiting_for_enter();
               
     speed = .5;
     turn = 1;
     
     //startup the teleoperation interface
     /*cout << "Reading from the keyboard  and Publishing to Twist!" << endl;
     cout << "---------------------------" << endl;
     cout << "Moving around:" << endl;
     cout << "u    i    o" << endl;
     cout << "j    k    l" << endl;
     cout << "m    ,    ." << endl;
     cout << " " << endl;
     cout << "q/z : increase/decrease max speeds by 10%" << endl;
     cout << "w/x : increase/decrease only linear speed by 10%" << endl;
     cout << "e/c : increase/decrease only angular speed by 10%" << endl;
     cout << "anything else : stop" << endl;  */   
     

     //main teleoperation server functionality
     int status = 0;
     while(1)
     {
/*             int key;
             char cmd_str[20*BUFSIZ];
     
             //(1) get the input key from the user
             key = getchar();
     
             //(2) translate the typed key to a motor command
             key_to_motor_cmd(key, cmd_str);   */
                         
             //(3) send a motor command to the server
     int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    portno = atoi("60050");
    //sockfd = socket(PF_INET, SOCK_RAW, IPPROTO_TCP);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname("192.168.0.3");
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(sockaddr*)&serv_addr,sizeof(serv_addr)) < 0) 
       error("ERROR connecting");
//       n = write(sockfd,cmd_str,strlen(cmd_str));
//       if (n < 0) 
//         error("ERROR writing to socket");
       
            //read data in chunks
            char image_info[57641];
            char image_buf[BUFSIZ];

            int cnt = 0;
            while(cnt < 57641 )
            {
                n = read(sockfd,image_buf, BUFSIZ);
                if (n < 0) 
                   error("ERROR reading from socket");
                memcpy(image_info + cnt, image_buf, n);  
                cnt = cnt + n;
            }

            //(5) deserialize and display info (image) on the screen
            boost::shared_ptr<sensor_msgs::Image> newim(new sensor_msgs::Image());            
            newim->deserialize((uint8_t *)image_info);  //receive data in chunks
            //newim->deserialize(image_info2);  //receive data all at once

            sensor_msgs::CvBridge bridge;
            try
            {
                cvShowImage("view", bridge.imgMsgToCv((const sensor_msgs::ImageConstPtr&)newim, "bgr8"));
            }
            catch(sensor_msgs::CvBridgeException& e)
            {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", newim->encoding.c_str());
            }

            close(sockfd);
            sleep(1);
 
      }//end while(1)
      
      cvDestroyWindow("view");
}



void key_to_motor_cmd(int key, char cmd_str[])
{
     float x, th, lx, az;
     x = 0;
     th = 0;

     switch(key)
     {
        case 'i': x = 1; th = 0; break;
        case 'o': x = 1; th = -1; break;
        case 'j': x = 0; th = 1; break;
        case 'l': x = 0; th = -1; break;
        case 'u': x = 1; th = 1; break;
        case ',': x = -1; th = 0; break;
        case '.': x = -1; th = 1; break;
        case 'm': x = -1; th = -1; break;
        case 'q': speed = speed*1.1; turn = turn*1.1; break;
        case 'z': speed = speed*0.9; turn = turn*0.9; break;
        case 'w': speed = speed*1.1; turn = turn*1; break;
        case 'x': speed = speed*0.9; turn = turn*1; break;
        case 'e': speed = speed*1; turn = turn*1.1; break;
        case 'c': speed = speed*1; turn = turn*0.9; break;
        default: x = 0; th = 0;
     }
     lx = x*speed;
     az = th*turn;
     
     cout << "speed = " << speed << ", turn = " << turn << ", x = " << x << ", th = " << th << endl;

     sprintf(cmd_str, "lx%4.7faz%4.7f", lx, az);     
}

void restore_terminal_settings()
{
     tcsetattr(0, TCSANOW, &oldt); //apply saved settings
}

void disable_waiting_for_enter()
{
    struct termios newt;
    //make terminal read 1 char at a time
    tcgetattr(0, &oldt); //save terminal settings
    newt = oldt;  //init new settings
    newt.c_lflag &= ~(ICANON | ECHO);  //change settings
    tcsetattr(0, TCSANOW, &newt);  //apply settings
    atexit(restore_terminal_settings); //make sure settings will be restored when program ends 

}

void error(char *msg)
{
    perror(msg);
    exit(1);
}
