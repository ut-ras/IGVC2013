#include "bumper_localizer_nomap.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "bumper_localizer_nomap");
	 ros::NodeHandle n1, n2, n3, n4, n5;
	 nhw = new NodeHandleWrapper();	 
     vel_pub = n1.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
     pos_sub = n2.subscribe("/position", 1, positionCallback);
     bump_sub = n4.subscribe("/sensorPacket", 1, bumperCallback);

	 //don't drive unless you are told so
     nhw->local_node_handle->param("step_navigator/ok_to_drive", ok_to_drive, 1);

	 ros::spin();
}

void positionCallback(const position_tracker::PositionConstPtr& msg)
{
    cur_pos = *msg;
} 

void bumperCallback(const irobot_create_2_1::SensorPacketConstPtr& msg)
{
    cur_sensors = *msg;

    nhw->local_node_handle->getParam("step_navigator/ok_to_drive", ok_to_drive);

    //we have recovered from bumping         
    if(!(cur_sensors.bumpLeft || cur_sensors.bumpRight))
    {		
        if(!ok_to_drive)
        {
            //ok_to_drive = 1;
		  nhw->local_node_handle->setParam("step_navigator/ok_to_drive", 1);

            prevBump = 0;

			//only useful when node running alone
		   /*geometry_msgs::Twist twist;
		   twist.angular.z = 0.0;
		   twist.linear.x = 0.0;
		   vel_pub.publish(twist);*/

        }
        
        return;
    }

   //ok_to_drive = 0;
   nhw->local_node_handle->setParam("step_navigator/ok_to_drive", 0);
   geometry_msgs::Twist twist;
   twist.angular.z = 0.0;
   twist.linear.x = 0.0;

   //we've hit something hard or head-on.  Back away slowly.
   if(cur_sensors.bumpLeft && cur_sensors.bumpRight)
   {
      twist.linear.x = -0.1; //-0.1;
      prevBump = 1;
   }
   else if(cur_sensors.bumpLeft)
   {   
      twist.angular.z = -0.1; //-0.1;
      prevBump = 1;
   }
   else if(cur_sensors.bumpRight)
   {
      twist.angular.z = 0.1;
      prevBump = 1;
   }

   cout << "Publish twist (" << twist.linear.x << ", " << twist.angular.z << ") from bumper_localizer" << endl;
   vel_pub.publish(twist);
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

