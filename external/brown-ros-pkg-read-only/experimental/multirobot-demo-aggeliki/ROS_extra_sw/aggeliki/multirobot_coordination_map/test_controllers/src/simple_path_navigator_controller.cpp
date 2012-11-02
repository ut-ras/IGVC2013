#include "simple_path_navigator_controller.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "simple_path_navigator_controller");
	 n1 = new ros::NodeHandle();

 	 //JUST FOR TESTING...
	 loadWaypoints();

	 cout << "Waypoints loaded" << endl;

	 //call the service of path_navigator to set the waypoints
	 ros::ServiceClient client = n1->serviceClient<path_navigator::setWaypoints>("set_waypoints");
     path_navigator::setWaypoints srv; 
	 path_navigator::Waypoints *w = new path_navigator::Waypoints();
	 w->waypoints = waypoints;           
     srv.request.w = *w;

     if(!client.call(srv))
     {
         cout << "Failed to call service set_waypoints" << endl;
     }

     ros::spin();     
}

void loadWaypoints()
{

   waypoints.clear();

   //create list with the waypoints
   /*map_loader::Node *p1 = new map_loader::Node();
   p1->p.x = 9;
   p1->p.y = 13;  
   waypoints.push_back(*p1); */
   map_loader::Node *p2 = new map_loader::Node();
   p2->p.x = 9;
   p2->p.y = 13;  
   waypoints.push_back(*p2);		
   map_loader::Node *p3 = new map_loader::Node();
   p3->p.x = 12;
   p3->p.y = 13;  
   waypoints.push_back(*p3);
   map_loader::Node *p4 = new map_loader::Node();
   p4->p.x = 15;
   p4->p.y = 13;  
   waypoints.push_back(*p4);
   waypoints.push_back(*p3); 
   waypoints.push_back(*p2); 
   //waypoints.push_back(*p1); 
}

