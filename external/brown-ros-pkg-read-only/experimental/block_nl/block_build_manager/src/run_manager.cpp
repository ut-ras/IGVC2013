
#include <ros/ros.h>
#include <block_build_manager/BuildManager.h>

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"build_manager");
  ros::NodeHandle nh;

  BuildManager bm(argc,argv);

  ROS_INFO("Build Manager initialized");
  bm.spin();
  ROS_INFO("Done");

  return 0;
}
