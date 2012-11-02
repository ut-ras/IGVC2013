
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_block_builder/Gripper.h>
#include <pr2_block_builder/RobotArm.h>
#include <pr2_block_builder/Head.h>
#include <simple_robot_control/torso_control.h>
#include <simple_robot_control/arm_control.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <pr2_block_builder_msgs/PickCommandAction.h>
#ifndef BLOCKGRABBER_H
#define BLOCKGRABBER_H

const std::string COMMAND_NAME = "/block_nl/pick";

class blockGrabber
{
public:
    blockGrabber();
    virtual ~blockGrabber();

    void executeASCB(const pr2_block_builder_msgs::PickCommandGoalConstPtr &goal);

    void initialize_robot();

    //For the toyblock_builder with controlled starting positions
    void grabNewBlock(int num);

    //For the toyblock_builder utilizing the tabletop manipulation stack
    int grabNewBlock();

    //For NLP/builder with given object's bounding box pose and dimensions.
//    int grabNewBlock(object_manager::Object object);

    geometry_msgs::Pose returnGraspPoseFromObjectPose(geometry_msgs::Pose pose, double rotation, bool lower);

    //Checks if an IK solution exists
    int checkIKSolutionExists(geometry_msgs::Pose pose, std::string link_name);

    void moveToHome();
    arm_navigation_msgs::MoveArmGoal returnGoal(int num, bool lower, std::string link_name);

    arm_navigation_msgs::MoveArmGoal returnGoal(geometry_msgs::Pose pose, bool lower, std::string link_name);

    object_manipulation_msgs::GraspableObject returnGraspableObject(tabletop_collision_map_processing::TabletopCollisionMapProcessingResponse processing_call_response);
    object_manipulation_msgs::Grasp returnGraspFromGraspObject(object_manipulation_msgs::GraspableObject grabObject);
    object_manipulation_msgs::Grasp returnGrasp(float x, float y, float z, float dir_x, float dir_y);

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
    simple_robot_control::Torso torso;
    static const float lookAt_x[];
    static const float lookAt_y[];
    static const float lookAt_z[];
};

#endif // BLOCKGRABBER_H
