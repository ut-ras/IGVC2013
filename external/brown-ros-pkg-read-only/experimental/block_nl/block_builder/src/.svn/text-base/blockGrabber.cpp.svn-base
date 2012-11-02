#include <block_builder/blockGrabber.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/CollisionOperation.h>
#include <arm_navigation_msgs/utils.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <tf/transform_datatypes.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <object_manager/Object.h>

#include <block_builder/Gripper.h>
#include <block_builder/RobotArm.h>
#include <block_build_msgs/PickCommandAction.h>
#include <simple_robot_control/torso_control.h>


#include <math.h>
#include <iostream>
#include <fstream>

blockGrabber::blockGrabber()
    : command_server(nh,COMMAND_NAME,boost::bind(&blockGrabber::executeASCB, this, _1), false),
      move_arm("move_right_arm",true)
{
    ROS_INFO("Waiting for move_arm server");
    move_arm.waitForServer();
    initialize_robot();
    command_server.start();
}

blockGrabber::~blockGrabber()
{

}

void blockGrabber::executeASCB(const block_build_msgs::PickCommandGoalConstPtr &goal)
{
    block_build_msgs::PickCommandResult result;
    result.result = false;

    if (goal->number == -3)
    {
        ROS_INFO("Moving robot to home position");
        initialize_robot();
        result.result = true;
    }
    else{
        ROS_INFO("Picking up block");
        int success_code = grabNewBlock(goal->object);

        if(command_server.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Goal Canceled");
            result.result = false;
            command_server.setSucceeded(result);
        }
        else if (success_code == 0)
        {
            ROS_INFO("Robot retrieved block");
            result.result = true;
        }
        else {
            ROS_INFO("Robot failed to retrieve block");
            result.result = false;
        }
    }
    //moveToHome();
    command_server.setSucceeded(result);
}

void blockGrabber::initialize_robot()
{
    ROS_INFO("Moving robot to start position");
    ROS_INFO("Opening gripper");
    gripper.openFull();
    ROS_INFO("Moving Torso");
    torso.move(0.3, true);
    ROS_INFO("Moving arms");
    moveToHome();
    ROS_INFO("Moving head");
    head.lookAt("torso_lift_link", 0.5,-0.1,-0.4);
    ROS_INFO("Robot is ready. Have fun!");
}

int blockGrabber::grabNewBlock(object_manager::Object object)
{
    if (object.pose.orientation.w == 0)
    {
        ROS_ERROR("Requested an undefined block");
        return -1;
    }
    std::cout << "Picking up block at location [x: "<< object.pose.position.x << " y: " << object.pose.position.y << " z: " << object.pose.position.z << " ]" << std::endl;
    gripper.openFull();
    float rotation;
    if (object.dim.x > object.dim.y)
    {
        rotation = 3.1415926535/2;
    }
    geometry_msgs::Pose pre_pose = returnGraspPoseFromObjectPose(object.pose, rotation, false);
    geometry_msgs::Pose goal_pose = returnGraspPoseFromObjectPose(object.pose, rotation, true);

    arm_navigation_msgs::MoveArmGoal goalA, goalB;
    goalA = returnGoal(pre_pose, false, "right_arm");
    goalB = returnGoal(goal_pose, true, "right_arm");
    ROS_INFO_STREAM("pre_pose: " << pre_pose);
    if (nh.ok())
    {
        bool finished_within_time = false;
        move_arm.sendGoal(goalA);
        finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
        if (!finished_within_time)
        {
            move_arm.cancelGoal();
            ROS_INFO("Timed out achieving goal A");
        }
        else
        {
            actionlib::SimpleClientGoalState state = move_arm.getState();
            bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
            if(success)
                ROS_INFO("Action finished: %s",state.toString().c_str());
            else
                ROS_INFO("Action failed: %s",state.toString().c_str());
        }
    }
    ROS_INFO_STREAM("goal_pose: " << goal_pose);
    if (nh.ok())
    {
        bool finished_within_time = false;
        move_arm.sendGoal(goalB);
        finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
        if (!finished_within_time)
        {
            move_arm.cancelGoal();
            ROS_INFO("Timed out achieving goal A");
        }
        else
        {
            actionlib::SimpleClientGoalState state = move_arm.getState();
            bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
            if(success)
                ROS_INFO("Action finished: %s",state.toString().c_str());
            else
                ROS_INFO("Action failed: %s",state.toString().c_str());
        }
    }
    bool closed = gripper.close();
    ROS_INFO_STREAM("post_pickup: " << pre_pose);
    if (nh.ok())
    {
        bool finished_within_time = false;
        move_arm.sendGoal(goalA);
        finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
        if (!finished_within_time)
        {
            move_arm.cancelGoal();
            ROS_INFO("Timed out achieving goal A");
        }
        else
        {
            actionlib::SimpleClientGoalState state = move_arm.getState();
            bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
            if(success)
                ROS_INFO("Action finished: %s",state.toString().c_str());
            else
                ROS_INFO("Action failed: %s",state.toString().c_str());
        }
    }
    if (closed)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

geometry_msgs::Pose blockGrabber::returnGraspPoseFromObjectPose(geometry_msgs::Pose pose, double rotation, bool lower)
{
    const std::string TABLE_SERVICE_NAME =
            "/blocknlp/request_table";

    //create service and action clients
    ros::ServiceClient table_srv;
    while ( !ros::service::waitForService(TABLE_SERVICE_NAME,
                                          ros::Duration(2.0)) && nh.ok() )
    {
        ROS_INFO("Waiting for table segmentation service to come up");
    }
    if (!nh.ok()) exit(0);
    table_srv =
            nh.serviceClient<object_manager::RequestTable>
            (TABLE_SERVICE_NAME, true);

    object_manager::RequestTable detection_call;
    if (!table_srv.call(detection_call))
    {
        ROS_ERROR("Tabletop detection service failed");
    }

    ROS_INFO_STREAM("Object pose: " << pose);
    ROS_INFO_STREAM("rotation: " << rotation);
    if (!lower)
    {
        pose.position.z += 0.1;
    }
    pose.position.z += 0.18;
    //    double roll = 3.1415926535/2.0;
    double pitch = 3.1415926535/2;
    //    double yaw = 0;
    tf::Quaternion q = tf::createQuaternionFromRPY(0, pitch, 0);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(q, q_msg);
    ROS_INFO_STREAM("q: " << q_msg);
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(pose.orientation, orientation);

    geometry_msgs::Pose goal_pose;
    goal_pose.position = pose.position;
    if (lower)
    {
        tf::quaternionTFToMsg(orientation * q, goal_pose.orientation);
    }
    else
    {
        tf::quaternionTFToMsg(orientation * q, goal_pose.orientation);
    }
    if(lower)
        ROS_INFO_STREAM("goal_pose.orientation: " << goal_pose.orientation);
    else
        ROS_INFO_STREAM("pre_pose.orientation: " << goal_pose.orientation);
    return goal_pose;

}

//returns 0 if true
//returns 1 if services can't be queried
//returns error_code.val if there is an IK problem
int blockGrabber::checkIKSolutionExists(geometry_msgs::Pose pose, std::string link_name)
{
    ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
    ros::service::waitForService("pr2_right_arm_kinematics/get_ik");

    ros::ServiceClient query_client = nh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
    ros::ServiceClient ik_client = nh.serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");

    // define the service messages
    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;

    if(query_client.call(request,response))
    {
        for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
        {
            ROS_DEBUG("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
        }
    }
    else
    {
        ROS_ERROR("Could not call query service");
        return -1;
    }

    kinematics_msgs::GetPositionIK::Request  gpik_req;
    kinematics_msgs::GetPositionIK::Response gpik_res;
    gpik_req.timeout = ros::Duration(5.0);
    gpik_req.ik_request.ik_link_name = link_name;

    gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
    gpik_req.ik_request.pose_stamped.pose = pose;
    gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
    gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
        gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
    }
    if(ik_client.call(gpik_req, gpik_res))
    {
        if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
        {
            for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
                ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
            return 0;
        }
        else
        {
            ROS_ERROR("Inverse kinematics failed with error code: %i", gpik_res.error_code.val);
            return gpik_res.error_code.val;
        }
    }
    else
    {
        ROS_ERROR("Inverse kinematics service call failed");
    }
}


void blockGrabber::moveToHome()
{
    arm.startTrajectory(arm.armExtensionTrajectory("right"), "right");
    // Wait for trajectory completion
    ROS_INFO("Waiting for right arm");
    while(!arm.getState("right").isDone() && ros::ok())
    {
        usleep(5000);
    }
    ROS_INFO("Move right arm complete");
    arm.startTrajectory(arm.armExtensionTrajectory("left"), "left");
    ROS_INFO("Waiting for left arm");
    while(!arm.getState("left").isDone() && ros::ok())
    {
        usleep(5000);
    }
    ROS_INFO("Move left arm complete");
}

arm_navigation_msgs::MoveArmGoal blockGrabber::returnGoal(geometry_msgs::Pose pose, bool lower, std::string arm_name)
{
    arm_navigation_msgs::MoveArmGoal goalA;

    arm_navigation_msgs::CollisionOperation op;
    op.object1 = op.COLLISION_SET_ALL;
    op.object2 = op.COLLISION_SET_ALL;
    op.operation = op.DISABLE;
    goalA.operations.collision_operations.push_back(op);

    goalA.motion_plan_request.group_name = arm_name;
    goalA.motion_plan_request.num_planning_attempts = 1;
    goalA.motion_plan_request.planner_id = std::string("");
    goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
    goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

    arm_navigation_msgs::SimplePoseConstraint desired_pose;
    desired_pose.header.frame_id = "base_link";
    desired_pose.link_name = "r_wrist_roll_link";

    desired_pose.pose = pose;

    desired_pose.absolute_position_tolerance.x = 0.02;
    desired_pose.absolute_position_tolerance.y = 0.02;
    desired_pose.absolute_position_tolerance.z = 0.02;
    desired_pose.absolute_roll_tolerance = 0.04;
    desired_pose.absolute_pitch_tolerance = 0.04;
    desired_pose.absolute_yaw_tolerance = 0.04;

    arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);
    return goalA;
}

