
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/CollisionOperation.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <block_build_msgs/PlaceCommandAction.h>
#include <object_manager/RequestTable.h>

#include <block_builder/Gripper.h>
#include <block_builder/RobotArm.h>

#ifndef _BLOCK_PLACER_H_
#define _BLOCK_PLACER_H_

const std::string OBJECT_DETECTION_SERVICE_NAME = "/object_detection";
const std::string COLLISION_PROCESSING_SERVICE_NAME = "/tabletop_collision_map_processing/tabletop_collision_map_processing";
const std::string PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup";
const std::string PLACE_ACTION_NAME = "/object_manipulator/object_manipulator_place";
const std::string COMMAND_NAME = "/blocknlp/place";

const std::string MOVE_ARM_NAME = "/move_right_arm";

class BlockPlacer
{
  private:
    ros::NodeHandle nh;
    ros::ServiceClient object_detection_srv;
    ros::ServiceClient collision_processing_srv;

    actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction> place_client;
    actionlib::SimpleActionServer<block_build_msgs::PlaceCommandAction> command_server;

    // arm naviation
    actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm;

    RobotArm ra;
    Gripper gripper;

  public:
    BlockPlacer(int argc, char** argv);
    ~BlockPlacer();

    void spin();
    void moveArmToInitPose();
    void moveArmToCenter(float x,float y,float z);
    void moveArmToCenter2(float x,float y,float z);
    void moveArmToPrePlacePose(float x,float y,float z, geometry_msgs::Quaternion orientation);
    void moveArmToPlacePose(float x,float y,float z, geometry_msgs::Quaternion orientation);
    void moveArmToPrecisePlacePose(float x, float y, float z, geometry_msgs::Quaternion orientation);

    void executeASCB(const block_build_msgs::PlaceCommandGoalConstPtr &goal);
    void sendGoal(arm_navigation_msgs::MoveArmGoal goalA);
    arm_navigation_msgs::MoveArmGoal createGoalMsg(float pose[]);
    pr2_controllers_msgs::JointTrajectoryGoal armStartTrajectory(float angles[]);

};

#endif
