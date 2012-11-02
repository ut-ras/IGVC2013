#include <pr2_block_builder/blockGrabber.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
//#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/CollisionOperation.h>
#include <arm_navigation_msgs/utils.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>

#include <pr2_block_builder/Gripper.h>
#include <pr2_block_builder/RobotArm.h>
#include <pr2_block_builder_msgs/PickCommandAction.h>
//#include <pr2_block_builder/Head.h>

blockGrabber::blockGrabber()
    : command_server(nh,COMMAND_NAME,boost::bind(&blockGrabber::executeASCB, this, _1), false),
      move_arm("move_right_arm",true)
{
    move_arm.waitForServer();
    command_server.start();
}

blockGrabber::~blockGrabber()
{

}

void blockGrabber::executeASCB(const pr2_block_builder_msgs::PickCommandGoalConstPtr &goal)
{
    ROS_INFO("Picking up block");
    pr2_block_builder_msgs::PickCommandResult result;
    //head.lookAt("base_link", 0.7, -0.7, -0.12);
    grabNewBlock(goal->number);
    //grabNewBlockAuto();
    if(command_server.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Goal Canceled");
        result.result = false;
        command_server.setSucceeded(result);
      }
      else {
        result.result = true;
      }

    //moveToHome();
    result.result = true;
    command_server.setSucceeded(result);
}

void blockGrabber::grabNewBlock(int num)
{
    gripper.openFull();
    //move_arm.disable_collision_monitoring = true;
    ROS_INFO("Connected to server");
    arm_navigation_msgs::MoveArmGoal goalA = returnGoal(num, false);
    arm_navigation_msgs::MoveArmGoal goalB = returnGoal(num, true);

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
    gripper.close();
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

int blockGrabber::grabNewBlockAuto()
{
    gripper.openFull();

    //set service and action names
    const std::string OBJECT_DETECTION_SERVICE_NAME =
            "/object_detection";
    const std::string COLLISION_PROCESSING_SERVICE_NAME =
            "/tabletop_collision_map_processing/tabletop_collision_map_processing";
    const std::string PICKUP_ACTION_NAME =
            "/object_manipulator/object_manipulator_pickup";

    //create service and action clients
    ros::ServiceClient object_detection_srv;
    ros::ServiceClient collision_processing_srv;
    actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction>
            pickup_client(PICKUP_ACTION_NAME, true);

    //wait for detection client
    while ( !ros::service::waitForService(OBJECT_DETECTION_SERVICE_NAME,
                                          ros::Duration(2.0)) && nh.ok() )
    {
        ROS_INFO("Waiting for object detection service to come up");
    }
    if (!nh.ok()) exit(0);
    object_detection_srv =
            nh.serviceClient<tabletop_object_detector::TabletopDetection>
            (OBJECT_DETECTION_SERVICE_NAME, true);

    //wait for collision map processing client
    while ( !ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME,
                                          ros::Duration(2.0)) && nh.ok() )
    {
        ROS_INFO("Waiting for collision processing service to come up");
    }
    if (!nh.ok()) exit(0);
    collision_processing_srv =
            nh.serviceClient
            <tabletop_collision_map_processing::TabletopCollisionMapProcessing>
            (COLLISION_PROCESSING_SERVICE_NAME, true);

    //wait for pickup client
    while(!pickup_client.waitForServer(ros::Duration(2.0)) && nh.ok())
    {
        ROS_INFO_STREAM("Waiting for action client " << PICKUP_ACTION_NAME);
    }
    if (!nh.ok()) exit(0);

    //call the tabletop detection
    ROS_INFO("Calling tabletop detector");
    tabletop_object_detector::TabletopDetection detection_call;
    //we want recognized database objects returned
    //set this to false if you are using the pipeline without the database
    detection_call.request.return_clusters = true;
    //we want the individual object point clouds returned as well
    detection_call.request.return_models = true;
    detection_call.request.num_models = 1;
    if (!object_detection_srv.call(detection_call))
    {
        ROS_ERROR("Tabletop detection service failed");
        return -1;
    }
    if (detection_call.response.detection.result !=
            detection_call.response.detection.SUCCESS)
    {
        ROS_ERROR("Tabletop detection returned error code %d",
                  detection_call.response.detection.result);
        return -1;
    }
    if (detection_call.response.detection.clusters.empty() &&
            detection_call.response.detection.models.empty() )
    {
        ROS_ERROR("The tabletop detector detected the table, "
                  "but found no objects");
        return -1;
    }

    //call collision map processing
    ROS_INFO("Calling collision map processing");
    tabletop_collision_map_processing::TabletopCollisionMapProcessing
            processing_call;
    //pass the result of the tabletop detection
    processing_call.request.detection_result =
            detection_call.response.detection;
    //ask for the existing map and collision models to be reset
    processing_call.request.reset_collision_models = true;
    processing_call.request.reset_attached_models = true;
    //ask for the results to be returned in base link frame
    processing_call.request.desired_frame = "base_link";
    if (!collision_processing_srv.call(processing_call))
    {
        ROS_ERROR("Collision map processing service failed");
        return -1;
    }
    //the collision map processor returns instances of graspable objects
    if (processing_call.response.graspable_objects.empty())
    {
        ROS_ERROR("Collision map processing returned no graspable objects");
        return -1;
    }

    //call object pickup
    ROS_INFO("Calling the pickup action");
    object_manipulation_msgs::PickupGoal pickup_goal;
    //pass one of the graspable objects returned
    //by the collision map processor
    pickup_goal.target = processing_call.response.graspable_objects.at(0);
    //pass the name that the object has in the collision environment
    //this name was also returned by the collision map processor
    pickup_goal.collision_object_name =
            processing_call.response.collision_object_names.at(0);
    //pass the collision name of the table, also returned by the collision
    //map processor
    pickup_goal.collision_support_surface_name =
            processing_call.response.collision_support_surface_name;
    //pick up the object with the right arm
    pickup_goal.arm_name = "right_arm";
    //we will be lifting the object along the "vertical" direction
    //which is along the z axis in the base_link frame
    geometry_msgs::Vector3Stamped direction;
    direction.header.stamp = ros::Time::now();
    direction.header.frame_id = "base_link";
    direction.vector.x = 0;
    direction.vector.y = 0;
    direction.vector.z = 1;
    pickup_goal.lift.direction = direction;
    //request a vertical lift of 10cm after grasping the object
    pickup_goal.lift.desired_distance = 0.1;
    pickup_goal.lift.min_distance = 0.05;
    //do not use tactile-based grasping or tactile-based lift
    pickup_goal.use_reactive_lift = false;
    pickup_goal.use_reactive_execution = false;
    //send the goal
    //pickup_goal.target.pose.position.z = -0.12;
    pickup_client.sendGoal(pickup_goal);
    while (!pickup_client.waitForResult(ros::Duration(10.0)))
    {
        ROS_INFO("Waiting for the pickup action...");
    }
    object_manipulation_msgs::PickupResult pickup_result =
            *(pickup_client.getResult());
    if (pickup_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR("The pickup action has failed with result code %d",
                  pickup_result.manipulation_result.value);
        return -1;
    }

    //remember where we picked the object up from
    geometry_msgs::PoseStamped pickup_location;
    //check if there are any matching database models (we only asked for the top
    // recognized model match, so there should only be 0 or 1 such)
    if (processing_call.response.graspable_objects.at(0).potential_models.size() > 0)
    {
        //for database recognized objects, the location of the object
        //is encapsulated in GraspableObject the message
        pickup_location =
                processing_call.response.graspable_objects.at(0).potential_models[0].pose;
    }
    else
    {
        //for unrecognized point clouds, the location of the object
        //is considered to be the origin of the frame that the
        //cluster is in
        pickup_location.header =
                processing_call.response.graspable_objects.at(0).cluster.header;
        //identity pose
        pickup_location.pose.orientation.w = 1;
    }
    return 0;
}

void blockGrabber::moveToHome()
{
    arm.startTrajectory(arm.armExtensionTrajectory());
    // Wait for trajectory completion
    while(!arm.getState().isDone() && ros::ok())
    {
        usleep(50000);
    }
}

arm_navigation_msgs::MoveArmGoal blockGrabber::returnGoal(int num, bool lower)
{
    double x_pos, y_pos, z_pos;
    y_pos = -0.4 - ((double)(num / 7 )/10);
    //y_pos = -0.4;
    if (lower)
    {
        x_pos = 0.432 + 0.044 * (num%7);
        z_pos = -0.12;
    }
    else
    {
        x_pos = 0.432 + 0.044 * (num%7) - 0.05;
        z_pos = 0.0;
    }
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
    desired_pose.header.frame_id = "torso_lift_link";
    desired_pose.link_name = "r_wrist_roll_link";
    desired_pose.pose.position.x = x_pos;
    desired_pose.pose.position.y = y_pos;
    desired_pose.pose.position.z = z_pos;

    desired_pose.pose.orientation.x = 0.0;
    desired_pose.pose.orientation.y = 0.707;
    desired_pose.pose.orientation.z = 0.0;
    desired_pose.pose.orientation.w = 0.707;

    desired_pose.absolute_position_tolerance.x = 0.02;
    desired_pose.absolute_position_tolerance.y = 0.02;
    desired_pose.absolute_position_tolerance.z = 0.02;

    desired_pose.absolute_roll_tolerance = 0.04;
    desired_pose.absolute_pitch_tolerance = 0.04;
    desired_pose.absolute_yaw_tolerance = 0.04;

    arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);
    return goalA;
}
