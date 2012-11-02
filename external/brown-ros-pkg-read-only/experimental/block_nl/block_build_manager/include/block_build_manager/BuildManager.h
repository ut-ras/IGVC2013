/*
  Author : Jihoon Lee
  Date   : Feb 2012

  BuildManager.h

*/


#include <ros/ros.h>
#include <simple_robot_control/robot_control.h>
#include <actionlib/server/simple_action_server.h>
#include <block_build_msgs/CommandAction.h>
#include <std_srvs/Empty.h>

#ifndef _BUILDMANAGER_H_
#define _BUILDMANAGER_H_


typedef simple_robot_control::Robot Robot;
typedef actionlib::SimpleActionServer<block_build_msgs::CommandAction> CMDServer;
const std::string COMMAND_NAME = "/blocknlp/command"; 

class BuildManager
{
  private:
    ros::NodeHandle nh;
    ros::ServiceClient seg_srv;
    Robot robot;
    CMDServer command_server;
    double right_arm_side[7];
    double left_arm_side[7];


  public:
    BuildManager(int argc, char** argv);
    ~BuildManager();

    void initialize();  // initialize robot's position such as torso, head, arms
    void scanTable();   // move head from left to right to see the whole table
    void segmentation(); // publish segmented point cloud

    void actionCallBack(const block_build_msgs::CommandGoalConstPtr &goal,CMDServer* command_server);
    void spin();

};

#endif
