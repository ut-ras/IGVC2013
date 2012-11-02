#include <iostream>
#include <vector>
#include <queue>
#include <costmap_2d/costmap_2d.h>
#include <navfn/navfn.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include "ardrone_lclzr/ardrone_mean_state.h"

using namespace std;
using namespace costmap_2d;
using namespace navfn;

//subscriber to move_base_msgs/MoveBaseGoal
//ros::Subscriber movebasegoal_sub;
//ros::Subscriber mapserver_sub;

//#define _ARDRONE_STARTX_ 12.55
//#define _ARDRONE_STARTY_ 41.88

#define _ARDRONE_STARTX_ 14.75
#define _ARDRONE_STARTY_ 41.88

//Butterworth low pass filter coefficients
/*
//fs=5Hz
//fc=0.75Hz
#define B1 0.1311
#define B2 0.2622
#define B3 0.1311
#define A1 1.0
#define A2 -0.7478
#define A3 0.2722
*/

/*
//fs=40Hz
//fc=0.1Hz
#define B1 0.000061
#define B2 0.000122
#define B3 0.000061
#define A1 1.0
#define A2 -1.9778
#define A3 0.9780
*/

/*
//fs=5Hz
//fc=0.75Hz
#define B1 0.3375
#define B2 0.3375
#define A1 1.0
#define A2 -0.3249
*/

//fs=40Hz
//fc=0.2Hz
#define B1 0.0155
#define B2 0.0155
#define A1 1.0
#define A2 -0.9691

/*
//fs=40Hz
//fc=0.1Hz
#define B1 0.0078
#define B2 0.0078
#define A1 1.0
#define A2 -0.9844
*/

//#define _ARDRONE_STARTX_ 8.21
//#define _ARDRONE_STARTY_ 41.885

//#define _ARDRONE_STARTX_ 6.85
//#define _ARDRONE_STARTY_ 43.05

ros::Publisher path_pub;

class ardrone_planner
{
public:

  //costmap_2d
  //a costmap instance subscribes to map_server
  Costmap2D *cmobj;

  //navfn
  NavFn *navfnobj;  

  //costmap initialized?
  bool cm_initialized;

  //navfn initialized?
  bool navfn_initialized;

  //costmap parameters
  unsigned int map_width;
  unsigned int map_height;
  double map_resolution;
  vector<unsigned char> map_data;
  double map_originx;
  double map_originy;
  double inradius;
  double circumradius;
  double inflationradius;
  double costscaling;
  double obstacle_range;
  double raytrace_range;
  double max_obstacle_height;
  int lethal_threshold;
  bool track_unknown_space;
  int unknown_cost_value;

  //NavFn parameters
  bool allow_unknown;
  double planner_window_x;
  double planner_window_y;
  double goal_tolerance;

  //create an instance of navfn
  ardrone_planner()
    :cmobj(NULL),navfnobj(NULL)
  {
    
    cm_initialized=false;
    navfn_initialized=false;
    
    //costmap parameters

    /*
    inradius=0.66; //for ardrone
    circumradius=0.66; //for ardrone
    inflationradius=0.60;
    */

    inradius=0.66; //for ardrone
    circumradius=0.7; //for ardrone
    inflationradius=0.75;

    costscaling=1.0;
    obstacle_range=2.5;
    raytrace_range=3.0;
    max_obstacle_height=2.0;
    lethal_threshold=100;
    track_unknown_space=false;
    unknown_cost_value=100;

    //NavFn parameters
    allow_unknown=true;
    planner_window_x=0.0;
    planner_window_y=0.0;
    goal_tolerance=0.0;

  }

  ~ardrone_planner()
  {
    if(cmobj)
      delete cmobj;

    if(navfnobj)
      delete navfnobj;
  }

};

ardrone_planner planner2d;

//queue<geometry_msgs::PoseStamped> ardrone_lclzr_poses;
double ardrone_lclzrx;
double ardrone_lclzry;

void initialize_navfn();

void mapserver_callback(const nav_msgs::OccupancyGridConstPtr& new_map)
{

  cout << "Inside mapserver_callback.\n";
  cout << "new_map->info.resolution: " << new_map->info.resolution << "\n";
  cout << "new_map->info.width: " << new_map->info.width << "\n";
  cout << "new_map->info.height: " << new_map->info.height << "\n";
  cout << "new_map->info.origin.position: (" << new_map->info.origin.position.x << ","
       << new_map->info.origin.position.y << "," 
       << new_map->info.origin.position.z << ")\n";
  cout << "new_map->info.origin.orientation: (" << new_map->info.origin.orientation.x 
       << "," << new_map->info.origin.orientation.y << "," 
       << new_map->info.origin.orientation.z << "," 
       << new_map->info.origin.orientation.w << ")\n";

  planner2d.map_width = new_map->info.width;
  planner2d.map_height = new_map->info.height;
  planner2d.map_resolution = new_map->info.resolution;
  planner2d.map_originx = new_map->info.origin.position.x;
  planner2d.map_originy = new_map->info.origin.position.y;
  unsigned int map_cellcount = planner2d.map_width * planner2d.map_height;
  planner2d.map_data.reserve(map_cellcount);
  for(unsigned int i=0; i<map_cellcount; i++)
    planner2d.map_data.push_back((unsigned char)new_map->data[i]);


  planner2d.cmobj = new Costmap2D(planner2d.map_width,planner2d.map_height, \
				  planner2d.map_resolution,planner2d.map_originx, \
				  planner2d.map_originy,planner2d.inradius, \
				  planner2d.circumradius,planner2d.inflationradius, \
				  planner2d.obstacle_range,planner2d.max_obstacle_height, \
				  planner2d.raytrace_range,planner2d.costscaling, \
				  planner2d.map_data,planner2d.lethal_threshold, \
				  planner2d.track_unknown_space, \
				  planner2d.unknown_cost_value);

  cout << "Completed costmap initialization.\n";
  cout << "costmap size in meters(x): " << planner2d.cmobj->getSizeInMetersX() 
       << "\n";
  cout << "costmap size in meters(y): " << planner2d.cmobj->getSizeInMetersY() 
       << "\n\n";
  cout << "costmap size in cells/pixels(x): " << planner2d.cmobj->getSizeInCellsX() 
       << "\n";
  cout << "costmap size in cells/pixels(y): " << planner2d.cmobj->getSizeInCellsY() 
       << "\n";

  planner2d.cm_initialized=true;

  initialize_navfn();

}

void initialize_navfn()
{
  //Initialize the navfn
  if(!planner2d.navfn_initialized && planner2d.cm_initialized)
    {
      planner2d.navfnobj = new NavFn(planner2d.cmobj->getSizeInCellsX(), \
				     planner2d.cmobj->getSizeInCellsY());
      planner2d.navfnobj->setCostmap(planner2d.cmobj->getCharMap(),true, \
				     planner2d.allow_unknown);
      planner2d.navfn_initialized=true;
    }
} 

//start and goal positions should be in the world frame
int makePlanDijkstra(double startx, double starty, double goalx, double goaly, vector<geometry_msgs::PoseStamped> &plan)
{
  if(!(planner2d.cm_initialized && planner2d.navfn_initialized))
    {
      cout << "costmap or planner have not been initialized. unable to plan.\n";
      return -1;
    }
  
  plan.clear();
  
  unsigned int mstartx, mstarty;
  if(!planner2d.cmobj->worldToMap(startx,starty,mstartx,mstarty))
    {
      cout << "robot start position is outside map area.\n";
      cout << "(startx,starty): (" << startx << "," << starty << ")\n";
      cout << "map coordinates of startx,starty: " << mstartx << "," << mstarty << "\n";
      return -1;
    }

  unsigned int mgoalx, mgoaly;
  if(!planner2d.cmobj->worldToMap(goalx,goaly,mgoalx,mgoaly))
    {
      cout << "robot goal position is outside map area.\n";
      cout << "(goalx,goaly): (" << goalx << "," << goaly << ")\n";
      cout << "map coordinates of goalx,goaly: " << mgoalx << "," << mgoaly << "\n";
      return -1;
    }

  int map_start[2];
  map_start[0]=mstartx;
  map_start[1]=mstarty;

  int map_goal[2];
  map_goal[0]=mgoalx;
  map_goal[1]=mgoaly;

  planner2d.navfnobj->setStart(map_start);
  planner2d.navfnobj->setGoal(map_goal);

  //compute the potential inplace in the costmap
  bool atStart=true;
  planner2d.navfnobj->calcNavFnDijkstra(atStart);

  //get plan from potential
  planner2d.navfnobj->setStart(map_start);  
  planner2d.navfnobj->calcPath(planner2d.cmobj->getSizeInCellsX() * 4);

  //extract the plan
  float *x = planner2d.navfnobj->getPathX();
  float *y = planner2d.navfnobj->getPathY();
  int len = planner2d.navfnobj->getPathLen();

  float xfilt[len];
  float yfilt[len];

  for(int i=0; i<len; i++)
    {
      xfilt[i]=x[i];
      yfilt[i]=y[i];
    }

  /*
  //Second order Butterworth filter
  if(len>2)
    {
      for(int i=2; i<len; i++)
	{
	  xfilt[i]=(B1*x[i]+B2*x[(i-1)]+B3*x[(i-2)]-A2*xfilt[(i-1)]-A3*xfilt[(i-2)]);
	  yfilt[i]=(B1*y[i]+B2*y[(i-1)]+B3*y[(i-2)]-A2*yfilt[(i-1)]-A3*yfilt[(i-2)]);
	}
    }
  */

  //First order Butterworth filter
  if(len>1)
    {
      for(int i=1; i<len; i++)
	{
	  xfilt[i]=(B1*x[i]+B2*x[(i-1)]-A2*xfilt[(i-1)]);
	  yfilt[i]=(B1*y[i]+B2*y[(i-1)]-A2*yfilt[(i-1)]);
	}
    }

  ros::Time plan_time = ros::Time::now();
  double worldx, worldy;
  for(int i=0; i<len; i++)
    {
      worldx = planner2d.cmobj->getOriginX() + xfilt[i] * planner2d.cmobj->getResolution();
      worldy = planner2d.cmobj->getOriginY() + yfilt[i] * planner2d.cmobj->getResolution();
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = "/map";
      pose.pose.position.x = worldx;
      pose.pose.position.y = worldy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }
      
  //publish plan for visualization
  nav_msgs::Path drone_path;
  drone_path.poses.resize(plan.size());
  
  drone_path.header.frame_id = plan[0].header.frame_id;
  drone_path.header.stamp = plan[0].header.stamp;
  
  for(unsigned int i=0; i < plan.size(); i++){
    drone_path.poses[i] = plan[i];
  }
  
  path_pub.publish(drone_path);

  return 1;
}

int makePlanAstar(double startx, double starty, double goalx, double goaly, vector<geometry_msgs::PoseStamped> &plan)
{
  if(!(planner2d.cm_initialized && planner2d.navfn_initialized))
    {
      cout << "costmap or planner have not been initialized. unable to plan.\n";
      return -1;
    }
  
  plan.clear();
  
  unsigned int mstartx, mstarty;
  if(!planner2d.cmobj->worldToMap(startx,starty,mstartx,mstarty))
    {
      cout << "robot start position is outside map area.\n";
      cout << "(startx,starty): (" << startx << "," << starty << ")\n";
      cout << "map coordinates of startx,starty: " << mstartx << "," << mstarty << "\n";
      return -1;
    }

  unsigned int mgoalx, mgoaly;
  if(!planner2d.cmobj->worldToMap(goalx,goaly,mgoalx,mgoaly))
    {
      cout << "robot goal position is outside map area.\n";
      cout << "(goalx,goaly): (" << goalx << "," << goaly << ")\n";
      cout << "map coordinates of goalx,goaly: " << mgoalx << "," << mgoaly << "\n";
      return -1;
    }

  int map_start[2];
  map_start[0]=mstartx;
  map_start[1]=mstarty;

  int map_goal[2];
  map_goal[0]=mgoalx;
  map_goal[1]=mgoaly;

  planner2d.navfnobj->setStart(map_start);
  planner2d.navfnobj->setGoal(map_goal);

  //compute the potential inplace in the costmap
  //bool atStart=true;
  //planner2d.navfnobj->calcNavFnDijkstra(atStart);
  planner2d.navfnobj->calcNavFnAstar();

  //get plan from potential
  planner2d.navfnobj->setStart(map_start);  
  planner2d.navfnobj->calcPath(planner2d.cmobj->getSizeInCellsX() * 4);

  //extract the plan
  float *x = planner2d.navfnobj->getPathX();
  float *y = planner2d.navfnobj->getPathY();
  int len = planner2d.navfnobj->getPathLen();
  
  ros::Time plan_time = ros::Time::now();
  double worldx, worldy;
  for(int i=0; i<len; i++)
    {
      worldx = planner2d.cmobj->getOriginX() + x[i] * planner2d.cmobj->getResolution();
      worldy = planner2d.cmobj->getOriginY() + y[i] * planner2d.cmobj->getResolution();
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = "/map";
      pose.pose.position.x = worldx;
      pose.pose.position.y = worldy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }
      
  //publish plan for visualization
  nav_msgs::Path drone_path;
  drone_path.poses.resize(plan.size());
  
  drone_path.header.frame_id = plan[0].header.frame_id;
  drone_path.header.stamp = plan[0].header.stamp;
  
  for(unsigned int i=0; i < plan.size(); i++){
    drone_path.poses[i] = plan[i];
  }
  
  path_pub.publish(drone_path);

  return 1;
}

void moveBaseCallback(const geometry_msgs::PoseStampedConstPtr& goalpose)
{
  double startx;
  double starty;
  double goalx;
  double goaly; 
  vector<geometry_msgs::PoseStamped> plan;
  startx = ardrone_lclzrx;
  starty = ardrone_lclzry;
  goalx = goalpose->pose.position.x;
  goaly = goalpose->pose.position.y;

  cout << "\nInside movebaseCallback.\n";
  cout << "(goalx,goaly): " << goalx << "," << goaly << "\n";

  //makePlan(startx, starty, goalx, goaly, plan);
  //makePlanAstar(startx, starty, goalx, goaly, plan);
  makePlanDijkstra(startx, starty, goalx, goaly, plan);

}

void ardrone_lclzr_callback(const ardrone_lclzr::ardrone_mean_state& drone_pose)
{
  //ardrone_lclzr_poses.push_back(drone_pose);
  ardrone_lclzrx=drone_pose.x;
  ardrone_lclzry=drone_pose.y;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ardrone_planner");
  ros::NodeHandle n;

  //the coordinates correspond to the center of lab 121
  //ardrone_lclzrx=6.85;
  //ardrone_lclzry=43.05;
  ///ardrone_lclzrx=8.21;
  ///ardrone_lclzry=41.885;
  ardrone_lclzrx=_ARDRONE_STARTX_;
  ardrone_lclzry=_ARDRONE_STARTY_;
  //ardrone_lclzrx=0.0;
  //ardrone_lclzry=0.0;

  ros::Subscriber mapsub = n.subscribe("/map", 0, &mapserver_callback);
  ros::Subscriber lclzr_sub = n.subscribe("/ardrone_mean_state",0,&ardrone_lclzr_callback);
  ros::Subscriber moveBaseSub = n.subscribe("/move_base_simple/goal",0,&moveBaseCallback);

  path_pub = n.advertise<nav_msgs::Path>("plan",1);

  ros::Rate loop_rate(10);
  //int i=0;
  while (ros::ok())
    {
      //cout << "i: " << i << "\n";
      //i++;
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}



