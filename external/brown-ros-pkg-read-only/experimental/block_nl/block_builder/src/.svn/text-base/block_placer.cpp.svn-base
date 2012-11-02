
#include <block_builder/block_placer.h>
#include <arm_navigation_msgs/utils.h>
#include <object_manager/Object.h>

BlockPlacer::BlockPlacer(int argc, char ** argv)
    : place_client(PLACE_ACTION_NAME, true),
      command_server(nh,COMMAND_NAME,boost::bind(&BlockPlacer::executeASCB, this, _1), false),
      move_arm(MOVE_ARM_NAME,true),
      ra(),
      gripper()
{
    while(!move_arm.waitForServer(ros::Duration(2.0)) && nh.ok())
    {
        ROS_INFO_STREAM("Waiting for action client " << MOVE_ARM_NAME);
    }

    if(!nh.ok()) exit(0);

}

BlockPlacer::~BlockPlacer()
{
}

void BlockPlacer::executeASCB(const block_build_msgs::PlaceCommandGoalConstPtr &goal)
{
    block_build_msgs::PlaceCommandResult result;

    //moveArmToCenter(goal->x,goal->y,goal->z);
    moveArmToPrePlacePose(goal->x,goal->y,goal->z, goal->orientation);

    if (goal->precise_place)
    {
        moveArmToPrecisePlacePose(goal->x, goal->y, goal->z, goal->orientation);
    }
    else
    {
        moveArmToPlacePose(goal->x,goal->y, goal->z, goal->orientation);
    }
    if(command_server.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("Goal Canceled");
        result.result = false;
        command_server.setSucceeded(result);
    }
    else {
        if (!goal->precise_place)
        {
            gripper.open();
        }
        gripper.openFull();
        result.result = true;
    }
    command_server.setSucceeded(result);
    moveArmToCenter2(goal->x,goal->y,goal->z);
    moveArmToInitPose();


}

void BlockPlacer::spin()
{
    command_server.start();
    ROS_INFO("command server has started");
    ros::spin();
}

void BlockPlacer::moveArmToInitPose()
{
    ra.startTrajectory(ra.armExtensionTrajectory("right"), "right");
    // Wait for trajectory completion
    ROS_INFO("Waiting for right arm");
    while(!ra.getState("right").isDone() && ros::ok())
    {
        usleep(50000);
    }

    ROS_INFO("Arm is at start pose");
}

void BlockPlacer::moveArmToCenter(float x,float y,float z)
{
    const float offset = 0.044;
    float pose[7] = {x,y,1.25,0,0.707,0,0.707};
    pose[2] = pose[2] + offset * z + 0.02;

    ROS_INFO_STREAM("X = " << pose[0] << " Y = " << pose[1] << " Z = " << pose[2] << " q.x: "<< pose[3] << " q.y: " << pose[4] << " q.z " << pose[5] << " q.w " << pose[6]);
    arm_navigation_msgs::MoveArmGoal goalA = createGoalMsg(pose);

    sendGoal(goalA);
}

void BlockPlacer::moveArmToCenter2(float x,float y,float z)
{
    const float offset = 0.044;
    float pose[7] = {x,y,1.25,0,0.707,0,0.707};
    pose[2] = pose[2] + offset * z + 0.02;

    ROS_INFO_STREAM("X = " << pose[0] << " Y = " << pose[1] << " Z = " << pose[2] << " q.x: "<< pose[3] << " q.y: " << pose[4] << " q.z " << pose[5] << " q.w " << pose[6]);
    arm_navigation_msgs::MoveArmGoal goalA = createGoalMsg(pose);

    sendGoal(goalA);
}

void BlockPlacer::moveArmToPrePlacePose(float x,float y,float z, geometry_msgs::Quaternion orientation)
{
    const float offset = 0.044;
    float pose[7] = {x,y,1.05,0,0.707,0,0.707};
    pose[2] = pose[2] + offset * z;
    pose[3] = orientation.x;
    pose[4] = orientation.y;
    pose[5] = orientation.z;
    pose[6] = orientation.w;

    ROS_INFO_STREAM("X = " << pose[0] << " Y = " << pose[1] << " Z = " << pose[2] << " q.x: "<< pose[3] << " q.y: " << pose[4] << " q.z " << pose[5] << " q.w " << pose[6]);
    arm_navigation_msgs::MoveArmGoal goalA = createGoalMsg(pose);

    sendGoal(goalA);
}

void BlockPlacer::moveArmToPlacePose(float x,float y,float z, geometry_msgs::Quaternion orientation)
{
    const float offset = 0.044;
    float pose[7] = {x,y,0.95,0,0.707,0,0.707};
    pose[2] = pose[2] + offset * z;
    pose[3] = orientation.x;
    pose[4] = orientation.y;
    pose[5] = orientation.z;
    pose[6] = orientation.w;

    ROS_INFO_STREAM("X = " << pose[0] << " Y = " << pose[1] << " Z = " << pose[2] << " q.x: "<< pose[3] << " q.y: " << pose[4] << " q.z " << pose[5] << " q.w " << pose[6]);
    arm_navigation_msgs::MoveArmGoal goalA = createGoalMsg(pose);

    sendGoal(goalA);
}

void BlockPlacer::moveArmToPrecisePlacePose(float x, float y, float z, geometry_msgs::Quaternion orientation)
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

    float pose[7] = {x,y, detection_call.response.table.pose.pose.position.z+0.2+0.044/2, orientation.x, orientation.y, orientation.z, orientation.w};

    ROS_INFO_STREAM("X = " << pose[0] << " Y = " << pose[1] << " Z = " << pose[2] << " q.x: "<< pose[3] << " q.y: " << pose[4] << " q.z " << pose[5] << " q.w " << pose[6]);
    arm_navigation_msgs::MoveArmGoal goalA = createGoalMsg(pose);

    sendGoal(goalA);
}

void BlockPlacer::sendGoal(arm_navigation_msgs::MoveArmGoal goalA)
{
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
}

arm_navigation_msgs::MoveArmGoal BlockPlacer::createGoalMsg(float pose[])
{
    arm_navigation_msgs::MoveArmGoal goalA;

    arm_navigation_msgs::CollisionOperation op;
    op.object1 = op.COLLISION_SET_ALL;
    op.object2 = op.COLLISION_SET_ALL;
    op.operation = op.DISABLE;

    goalA.operations.collision_operations.push_back(op);
    goalA.motion_plan_request.group_name = "right_arm";
    goalA.motion_plan_request.num_planning_attempts = 1;
    goalA.motion_plan_request.planner_id = std::string("");
    goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
    goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

    arm_navigation_msgs::SimplePoseConstraint desired_pose;
    //desired_pose.header.frame_id = "torso_lift_link";
    desired_pose.header.frame_id = "base_link";
    desired_pose.link_name = "r_wrist_roll_link";
    desired_pose.pose.position.x = pose[0];
    desired_pose.pose.position.y = pose[1];
    desired_pose.pose.position.z = pose[2];

    desired_pose.pose.orientation.x = pose[3];
    desired_pose.pose.orientation.y = pose[4];
    desired_pose.pose.orientation.z = pose[5];
    desired_pose.pose.orientation.w = pose[6];

    desired_pose.absolute_position_tolerance.x = 0.02;
    desired_pose.absolute_position_tolerance.y = 0.02;
    desired_pose.absolute_position_tolerance.z = 0.02;

    desired_pose.absolute_roll_tolerance = 0.04;
    desired_pose.absolute_pitch_tolerance = 0.04;
    desired_pose.absolute_yaw_tolerance = 0.04;

    arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

    return goalA;
}

pr2_controllers_msgs::JointTrajectoryGoal BlockPlacer::armStartTrajectory(float angles[])
{
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    goal.trajectory.points.resize(1);
    // Second trajectory point
    // Positions
    int ind= 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = angles[0];
    goal.trajectory.points[ind].positions[1] = angles[1];
    goal.trajectory.points[ind].positions[2] = angles[2];
    goal.trajectory.points[ind].positions[3] = angles[3];
    goal.trajectory.points[ind].positions[4] = angles[4];
    goal.trajectory.points[ind].positions[5] = angles[5];
    goal.trajectory.points[ind].positions[6] = angles[6];
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
        goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(6.0);

    //we are done; return the goal
    return goal;

}
