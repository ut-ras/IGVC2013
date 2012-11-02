#include <pr2_block_builder/blockGrabber.h>
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

#include <pr2_block_builder/Gripper.h>
#include <pr2_block_builder/RobotArm.h>
#include <pr2_block_builder_msgs/PickCommandAction.h>
#include <simple_robot_control/torso_control.h>


#include <math.h>
#include <iostream>
#include <fstream>


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
    pr2_block_builder_msgs::PickCommandResult result;
    result.result = true;
    switch(goal->number)
    {
    case -1:
        grabNewBlock();
        break;
    case -3:
        ROS_INFO("Moving robot to home position");
        initialize_robot();
        break;
    default:
        grabNewBlock(goal->number);
    }
    command_server.setSucceeded(result);
}

const float blockGrabber::lookAt_x[] = {0.5, 0.8};
const float blockGrabber::lookAt_y[] = {0.2, 0.5};
const float blockGrabber::lookAt_z[] = {-0.12, -0.12};

void blockGrabber::initialize_robot()
{
    torso.move(0.3, true);
    moveToHome();
    head.lookAt("torso_lift_link", 0.5,-0.1,-0.4);
}

void blockGrabber::grabNewBlock(int num)
{
    std::cout << "Picking up block number: " << num << std::endl;
    gripper.openFull();
    //move_arm.disable_collision_monitoring = true;
    ROS_INFO("Connected to server");
    arm_navigation_msgs::MoveArmGoal goalA = returnGoal(num, false, "right_arm");
    arm_navigation_msgs::MoveArmGoal goalB = returnGoal(num, true, "right_arm");

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

int blockGrabber::grabNewBlock()
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
    int i = 0;
    bool objects_not_found = true;
    tabletop_object_detector::TabletopDetection detection_call;
    tabletop_collision_map_processing::TabletopCollisionMapProcessing
            processing_call;
    while (i<2 && objects_not_found)
    {
        head.lookAt("base_link", lookAt_x[i], lookAt_y[i] , lookAt_z[i]);

        ROS_INFO("Calling tabletop detector");
        //we want recognized database objects returned
        //set this to false if you are using the pipeline without the database
        detection_call.request.return_clusters = true;
        //we want the individual object point clouds returned as well
        detection_call.request.return_models = false;
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
            ROS_INFO("Didn't find objects, checking new location");
            i++;
            if (i==3)
            {
                ROS_ERROR("No objects detected");
                return 1;
            }
        }
    }

    //call object pickup
    ROS_INFO("Calling the pickup action");
    object_manipulation_msgs::PickupGoal pickup_goal;
    //pass one of the graspable objects returned
    //by the collision map processor

    pickup_goal.target = returnGraspableObject(processing_call.response);
    pickup_goal.desired_grasps.push_back(returnGraspFromGraspObject(pickup_goal.target));

    //pickup_goal.target = processing_call.response.graspable_objects.at(0);
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

    return 0;
}

geometry_msgs::Pose blockGrabber::returnGraspPoseFromObjectPose(geometry_msgs::Pose pose, double rotation, bool lower)
{
    ROS_INFO("Object's pose x: %f y: %f z: %f q.x: %f q.y %f q.z %f q.w %f", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    ROS_INFO("Rotation: %f", rotation);
    if (!lower)
    {
        pose.position.z += 0.1;
    }
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(pose.orientation, orientation);

    geometry_msgs::Pose goal_pose;
    goal_pose.position = pose.position;
    tf::quaternionTFToMsg(q * orientation, goal_pose.orientation);

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

    gpik_req.ik_request.pose_stamped.header.frame_id = "base_link";
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
            for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
                ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
        else
        {
            ROS_ERROR("Inverse kinematics failed");
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
    arm.startTrajectory(arm.armExtensionTrajectory());
    // Wait for trajectory completion
    while(!arm.getState().isDone() && ros::ok())
    {
        usleep(50000);
    }
}

arm_navigation_msgs::MoveArmGoal blockGrabber::returnGoal(int num, bool lower, std::string arm_name)
{
    double x_pos, y_pos, z_pos;
    int row, column, layer;

    layer  = ( num / 7) / 2;
    column = ( num / 7) % 2;
    row    =   num % 7;

    x_pos =  0.45 + 0.044 * row;
    y_pos = -0.40 - 0.1   * column;
    z_pos =  1.05 + 0.045 * layer;

    if (!lower)
    {
        x_pos -= 0.05;
        z_pos += 0.1;
    }

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
    desired_pose.link_name = arm_name.compare("right_arm") ? "r_wrist_roll_link" : "l_wrist_roll_link";
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

arm_navigation_msgs::MoveArmGoal blockGrabber::returnGoal(geometry_msgs::Pose pose, bool lower, std::string arm_name)
{
    arm_navigation_msgs::MoveArmGoal goalA;

    if (lower)
    {
        arm_navigation_msgs::CollisionOperation op;
        op.object1 = op.COLLISION_SET_ALL;
        op.object2 = op.COLLISION_SET_ALL;
        op.operation = op.DISABLE;
        goalA.operations.collision_operations.push_back(op);
    }

    goalA.motion_plan_request.group_name = arm_name;
    goalA.motion_plan_request.num_planning_attempts = 1;
    goalA.motion_plan_request.planner_id = std::string("");
    goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
    goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

    arm_navigation_msgs::SimplePoseConstraint desired_pose;
    desired_pose.header.frame_id = "base_link";
    desired_pose.link_name = arm_name.compare("right_arm") ? "r_wrist_roll_link" : "l_wrist_roll_link";

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


object_manipulation_msgs::GraspableObject blockGrabber::returnGraspableObject(tabletop_collision_map_processing::TabletopCollisionMapProcessingResponse processing_call_response)
{
    std::ofstream pointsfile;
    pointsfile.open("/home/brawner/points.csv", std::ios::out | std::ios::app);
    int length = processing_call_response.graspable_objects.size();
    object_manipulation_msgs::GraspableObject grabObject = processing_call_response.graspable_objects.at(0);

    for (int i = 0; i<length; i++)
    {
        if (grabObject.cluster.points.size() < processing_call_response.graspable_objects.at(i).cluster.points.size())
        {
            grabObject = processing_call_response.graspable_objects.at(i);
        }
    }
    pointsfile.close();
    return grabObject;
}


object_manipulation_msgs::Grasp blockGrabber::returnGraspFromGraspObject(object_manipulation_msgs::GraspableObject grabObject)
{
    float x,y,z;
    float mean_x = 0;
    float mean_y = 0;
    int *bins = new int[60];
    memset(bins, 0, 60 * sizeof(int));
    std::vector<int> bin_selections;
    int bin_selection;
    float length = grabObject.cluster.points.size();
    std::cout << "length: " << length << std::endl;
    for (int i = 0; i<length; i++)
    {
        x = grabObject.cluster.points.at(i).x;
        y = grabObject.cluster.points.at(i).y;
        z = grabObject.cluster.points.at(i).z;
        bin_selection = (z-0.7)/0.001;
        bin_selections.push_back(bin_selection);
        bins[bin_selection]++;
        //std::cout << "z: " << z << " bin_selection: " << bin_selection << std::endl;
    }
    int max = -1;
    int max_index = -1;
    for (int i = 0; i<60; i++)
    {
        std::cout << "bins[i]: " << bins[i] << std::endl;
        if (bins[i]>max)
        {
            max = bins[i];
            max_index = i;
        }
    }
    z = max_index * 0.001 + 0.7 - 0.005;

    for (int i = 0; i<length; i++)
    {
        //std::cout << "max_index: " << max_index << bin_selections.at(i) << std::endl;
        if (bin_selections.at(i) == max_index)
        {
            mean_x += grabObject.cluster.points.at(i).x;
            mean_y += grabObject.cluster.points.at(i).y;
            std::cout << " point: " << grabObject.cluster.points.at(i).x << ", " << grabObject.cluster.points.at(i).y << std::endl;
        }
    }

    float *corner_dist = new float[4];
    float *corner_x = new float[4];
    float *corner_y = new float[4];

    float distance, distX, distY;
    bool isValidCorner;

    for (int i = 0; i<4; i++)
    {
        for (int j = 0; j<length; j++)
        {
            if (bin_selections.at(j) == max_index)
            {
                distX = grabObject.cluster.points.at(i).x - x;
                distY = grabObject.cluster.points.at(i).y - y;
                distance = distX*distX + distY * distY;
                isValidCorner = true;
                for (int k = 0; k < i; k++)
                {
                    if (distX * corner_x[i] + distY * corner_y[i] > 0.3 * distance)
                    {
                        isValidCorner = false;
                    }
                }

                if (distance > corner_dist[i] && isValidCorner)
                {
                    corner_dist[i] = distance;
                    corner_x[i] = distX;
                    corner_y[i] = distY;
                }
            }
        }
    }
    x = (corner_x[0] + corner_x[1] + corner_x[2] + corner_x[3])/4.0;
    y = (corner_y[0] + corner_y[1] + corner_y[2] + corner_y[3])/4.0;
    std::cout << "Corner 0: " << corner_x[0] << ", " << corner_y[0] << " Corner1: " << corner_x[1] << ", " << corner_y[1];
    std::cout <<" Corner 2: " << corner_x[2] << ", " << corner_y[2] << " Corner3: " << corner_x[3] << ", " << corner_y[3];
    distance = 0;
    float *sorted_corner_x = new float[4];
    float *sorted_corner_y = new float[4];
    memcpy(sorted_corner_x, corner_x, 4*sizeof(float));
    memcpy(sorted_corner_y, corner_y, 4*sizeof(float));
    float max_distance = 0;
    for (int i = 1; i<4; i++)
    {
        distance = (corner_x[0]-corner_x[i])*(corner_x[0]-corner_x[i]) + (corner_y[0] - corner_y[i])*(corner_y[0] - corner_y[i]);
        if (distance > max_distance)
        {
            max_distance = distance;
            sorted_corner_x[i] = corner_x[1];
            sorted_corner_y[i] = corner_y[1];
            sorted_corner_x[1] = corner_x[i];
            sorted_corner_y[1] = corner_y[i];
        }
    }
    float dir_x = (sorted_corner_x[0] + sorted_corner_x[1])/2.0 - x;
    float dir_y = (sorted_corner_y[0] + sorted_corner_y[1])/2.0 - y;

    std::cout << "x: " << x << " y: " << y << " z: " << z << " dir_x: " << dir_x << " dir_y: " << dir_y << std::endl;
    return returnGrasp(x,y,z,dir_x,dir_y);
}

object_manipulation_msgs::Grasp blockGrabber::returnGrasp(float x_pos, float y_pos, float z_pos, float dir_x, float dir_y)
{
    //    double roll = 3.1415926535/2.0;
    double pitch = 3.1415926535/2;
    double yaw = acos((dir_x * -1.0 + dir_y * 0) / (dir_x * dir_x + dir_y * dir_y));
    tf::Quaternion q = tf::createQuaternionFromRPY(0, pitch, 0);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(q, q_msg);
    ROS_INFO_STREAM("q: " << q_msg);

    object_manipulation_msgs::Grasp grasp;
    sensor_msgs::JointState pre_grasp_posture;
    sensor_msgs::JointState grasp_posture;
    geometry_msgs::Pose grasp_pose;

    pre_grasp_posture.position.push_back(100);
    grasp_posture.position.push_back(0);
    grasp_posture.effort.push_back(100);

    grasp.success_probability = 1.0;
    grasp.cluster_rep = true;

    pre_grasp_posture.header.frame_id = "base_link";
    grasp_posture.header.frame_id = "base_link";
    grasp_pose.position.x = x_pos;
    grasp_pose.position.y = y_pos;
    grasp_pose.position.z = z_pos;

    grasp_pose.orientation = q_msg;

    grasp.grasp_pose = grasp_pose;
    grasp.pre_grasp_posture = pre_grasp_posture;
    grasp.grasp_posture = grasp_posture;

    return grasp;
}

