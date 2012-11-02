
#include <ros/ros.h>
#include <object_manager/ObjectManager.h>

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"object_manager");
  ros::NodeHandle nh;

  ObjectManager om(argc,argv);

  ROS_INFO("Object Manager initialized");
  om.spin();
  ROS_INFO("Done");

  return 0;
}
