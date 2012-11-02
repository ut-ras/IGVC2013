
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_block_builder/Gripper.h>
#include <pr2_block_builder/RobotArm.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <pr2_block_builder_msgs/PickCommandAction.h>
#include <pr2_block_builder/Head.h>
#ifndef BLOCKGRABBER_H
#define BLOCKGRABBER_H

const std::string COMMAND_NAME = "/pr2_block_builder/pick";

class blockGrabber
{
public:
    blockGrabber();
    virtual ~blockGrabber();

    void executeASCB(const pr2_block_builder_msgs::PickCommandGoalConstPtr &goal);
    void grabNewBlock(int num);
    int grabNewBlockAuto();
    void moveToHome();
    arm_navigation_msgs::MoveArmGoal returnGoal(int num, bool lower);

private:
    ros::NodeHandle nh;
    ros::ServiceClient object_detection_srv;
    ros::ServiceClient collision_processing_srv;

    // arm naviation
    actionlib::SimpleActionServer<pr2_block_builder_msgs::PickCommandAction> command_server;
    actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm;

    RobotArm arm;
    Gripper gripper;
    Head head;

};

#endif // BLOCKGRABBER_H
