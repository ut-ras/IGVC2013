
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <block_builder/Gripper.h>
#include <block_builder/RobotArm.h>
#include <block_builder/Head.h>
#include <simple_robot_control/torso_control.h>
#include <simple_robot_control/arm_control.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <block_build_msgs/PickCommandAction.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <object_manager/Object.h>
#include <object_manager/RequestTable.h>
#ifndef BLOCKGRABBER_H
#define BLOCKGRABBER_H

const std::string COMMAND_NAME = "/blocknlp/pick";

class blockGrabber
{
public:
    blockGrabber();
    virtual ~blockGrabber();

    void executeASCB(const block_build_msgs::PickCommandGoalConstPtr &goal);

    //For NLP/builder with given object's bounding box pose and dimensions.
    int grabNewBlock(object_manager::Object object);

    geometry_msgs::Pose returnGraspPoseFromObjectPose(geometry_msgs::Pose pose, double rotation, bool lower);

    //Checks if an IK solution exists
    int checkIKSolutionExists(geometry_msgs::Pose pose, std::string link_name);

    void moveToHome();

    arm_navigation_msgs::MoveArmGoal returnGoal(geometry_msgs::Pose pose, bool lower, std::string link_name);

private:
    void initialize_robot();
    ros::NodeHandle nh;
    ros::ServiceClient object_detection_srv;
    ros::ServiceClient collision_processing_srv;

    // arm naviation
    actionlib::SimpleActionServer<block_build_msgs::PickCommandAction> command_server;
    actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm;

    RobotArm arm;
    Gripper gripper;
    Head head;
    simple_robot_control::Torso torso;

};

#endif // BLOCKGRABBER_H
