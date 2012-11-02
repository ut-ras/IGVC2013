#include "simple_path_generator_controller_dijkstras.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "simple_path_generator_controller_dijkstras");
	 n1 = new ros::NodeHandle();
	 ros::Subscriber pos_sub = n1->subscribe("/position", 1, positionCallback);

 	 //JUST FOR TESTING... load a shortest path between the current location of the robot and a destination location
	 loadDijkstrasWaypoints();

	 cout << "Waypoints loaded" << endl;

	 //FOR NOW, just display the waypoints you generated
	 vector<map_loader::Node>::iterator it;
     cout << "Generated waypoints:" << endl;
	 int cnt = 1;
	 for(it = waypoints.begin(); it!=waypoints.end(); ++it)
	 {
		 cout << "waypoint: x = " << it->p.x << ", y = " << it->p.y << endl;
	 }

	/* //call the service of path_navigator to set the waypoints
	 ros::ServiceClient client = n1->serviceClient<path_navigator::setWaypoints>("set_waypoints");
     path_navigator::setWaypoints srv; 
	 path_navigator::Waypoints *w = new path_navigator::Waypoints();
	 w->waypoints = waypoints;           
     srv.request.w = *w;

     if(!client.call(srv))
     {
         cout << "Failed to call service set_waypoints" << endl;
     }*/

     ros::spin();     
}

void positionCallback(const position_tracker::PositionConstPtr& msg)
{
     cur_pos = *msg;
}

void loadDijkstrasWaypoints()
{
  	 ros::ServiceClient client = n1->serviceClient<path_generator::generatePath>("generate_path");
     path_generator::generatePath srv; 
	 path_navigator::Waypoints *w = new path_navigator::Waypoints();
	 w->waypoints = waypoints;           
     srv.request.type = 0;
	 //TESTING... set your cur_pos through code... (then checkout the waypoints topic for the results) -> play with different values of cur_pos and dst_pos
	 position_tracker::Position *init_pos = new position_tracker::Position();
     init_pos->x = 7;
	 init_pos->y = 12.7;

     srv.request.init_pos = *init_pos;
     //srv.request.init_pos = cur_pos;
	 position_tracker::Position *dst_pos = new position_tracker::Position();
	 dst_pos->x = 15;
	 dst_pos->y = 11;
     srv.request.dst_pos = *dst_pos;

     if(!client.call(srv))
     {
         cout << "Failed to call service set_waypoints" << endl;
     }
 
	 vector<map_loader::Node>::iterator it;
     cout << "JUST generated waypoints:" << endl;
	 int cnt = 1;
	 for(it = srv.response.w.waypoints.begin(); it!=srv.response.w.waypoints.end(); ++it)
	 {
		 cout << "waypoint: x = " << it->p.x << ", y = " << it->p.y << endl;
	 }

     waypoints = srv.response.w.waypoints;  

	//TESTING AGAIN...
     	 init_pos->x = 15;
	 init_pos->y = 11;
         srv.request.init_pos = *init_pos;
        //srv.request.init_pos = cur_pos;
	 dst_pos->x = 10;
	 dst_pos->y = 12.7;
         srv.request.dst_pos = *dst_pos;

     	 if(!client.call(srv))
     	 {
            cout << "Failed to call service set_waypoints" << endl;
         }
 
    	 cout << "JUST generated waypoints (again):" << endl;
	 for(it = srv.response.w.waypoints.begin(); it!=srv.response.w.waypoints.end(); ++it)
	 {
		 cout << "waypoint: x = " << it->p.x << ", y = " << it->p.y << endl;
	 }

}

