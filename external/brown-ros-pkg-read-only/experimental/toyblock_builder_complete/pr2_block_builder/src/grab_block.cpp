#include <ros/ros.h>
//#include <actionlib/client/simple_action_client.h>
//#include <pr2_controllers_msgs/JointTrajectoryAction.h>
//#include <arm_navigation_msgs/MoveArmAction.h>
//#include <arm_navigation_msgs/CollisionOperation.h>
//#include <arm_navigation_msgs/utils.h>
//#include <tabletop_object_detector/TabletopDetection.h>
//#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
//#include <object_manipulation_msgs/PickupAction.h>
//#include <object_manipulation_msgs/PlaceAction.h>

//#include <pr2_block_builder/Gripper.h>
//#include <pr2_block_builder/RobotArm.h>
#include <pr2_block_builder/blockGrabber.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>


int main(int argc, char **argv)
{
    //initialize the ROS node
    ros::init(argc, argv, "pick_and_place_app");
    ros::NodeHandle nh;
    blockGrabber grabber;

    grabber.moveToHome();
    ros::spin();    
}
