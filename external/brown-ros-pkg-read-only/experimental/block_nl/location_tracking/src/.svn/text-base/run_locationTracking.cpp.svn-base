
#include <location_tracking/locationTracker.h>

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"locationTracker");
  ros::NodeHandle nh;
  locationTracker *lt = new locationTracker(argc,argv);
  ROS_INFO("Location Tracker Initialized");
  nh->spin();
  delete lt;
  return 0;
}
