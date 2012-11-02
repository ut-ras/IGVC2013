/*
  Author : Jihoon Lee
  Date   : Feb. 2012

  BuildManger.cpp
  */

#include<block_build_manager/BuildManager.h>

BuildManager::BuildManager(int argc, char** argv)
  : robot(),
    command_server(nh,COMMAND_NAME,boost::bind(&BuildManager::actionCallBack,this,_1,&(this->command_server)),false)
{
/*  while (!ros::service::waitForService(service_name, ros::Duration(2.0)) && nh.ok())
  {
//    ROS_INFO("Waiting for %s service to come up",service_name.c_str());
  }
  if(!nh.ok()) exit(0);
  seg_srv =nh.serviceClient<std_srvs::Empty>(service_name,true);
*/
  right_arm_side[0] = -1.43;
  right_arm_side[1] = -0.35;
  right_arm_side[2] = -1.51;
  right_arm_side[3] = -1.47;
  right_arm_side[4] = 5.05;
  right_arm_side[5] = -1.65;
  right_arm_side[6] = -3.14;

  left_arm_side[0] = 1.43;
  left_arm_side[1] = -0.35;
  left_arm_side[2] = 1.51;
  left_arm_side[3] = -1.47;
  left_arm_side[4] = -5.05;
  left_arm_side[5] = -1.65;
  left_arm_side[6] = -3.14;
}

BuildManager::~BuildManager()
{
}

void BuildManager::initialize()
{
  ROS_INFO("Move Torso");
  // elevate the torso all the way up.
  robot.torso.move(0.3,true);

  ROS_INFO("Move Head");
  // head control
  robot.head.lookat("torso_lift_link", tf::Vector3(0.5,-0.1,-0.4));

  ROS_INFO("Move right Arm");
  // move right arm to the sides
  std::vector<double> rside_pos(right_arm_side,right_arm_side+7);
  robot.right_arm.goToJointPos(rside_pos);

  ROS_INFO("Move Left Arm");
  // move left arm to the sides
  std::vector<double> lside_pos(left_arm_side,left_arm_side+7);
  robot.left_arm.goToJointPos(lside_pos);

}

void BuildManager::actionCallBack(const block_build_msgs::CommandGoalConstPtr &goal,CMDServer* server)
{
  ROS_INFO("In Action Callback");
  bool hey = true;
  int cnt = 0;

  switch(goal->command)
  {
    case 1:
      while(hey && !server->isPreemptRequested())
      {
        cnt++;
        ROS_INFO_STREAM("cnt = " << cnt);
      }
      break;
    case 2:
      ROS_INFO("X = 2");
      server->setPreempted();
      //hey = false;

      break;
    case 3:
      initialize();
      ROS_INFO("Pose initialized");
      break;
    case 4:
      scanTable();
      ROS_INFO("Scan table");
      break;
    case 5:
      segmentation();
      break;
    default:
      ROS_INFO("X = default");
      break;
  }

  ROS_INFO("Here");

  block_build_msgs::CommandResult result;
  result.result = true;
  server->setSucceeded(result);
}

void BuildManager::scanTable()
{
  // look right
  robot.head.lookat("torso_lift_link", tf::Vector3(0.5,-0.6,-0.1),1);
  ros::Duration(1.0).sleep();
  // look left
  robot.head.lookat("torso_lift_link", tf::Vector3(0.5,-0.1,-0.4),0.1);
}

void BuildManager::segmentation()
{
}

void BuildManager::spin()
{
  command_server.start();
  ros::spin();
}

