#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <arm_navigation_msgs/MoveArmAction.h>

#ifndef ROBOTARM_H
#define ROBOTARM_H

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class RobotArm
{
private:
    // Action client for the joint trajectory action
    // used to trigger the arm movement action
    TrajClient* r_traj_client_;
    TrajClient* l_traj_client_;

public:
    //! Initialize the action client and wait for action server to come up
    RobotArm()
    {
        // tell the action client that we want to spin a thread by default
        r_traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
        l_traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);


        // wait for action server to come up
        ROS_INFO("Waiting for the right joint_trajectory_action server");
        while(!r_traj_client_->waitForServer(ros::Duration(5.0))){
            usleep(50000);
        }
        ROS_INFO("Waiting for the left joint_trajectory_action server");
        while(!l_traj_client_->waitForServer(ros::Duration(5.0))){
            usleep(50000);
        }

    }

    //! Clean up the action client
    ~RobotArm()
    {
        delete r_traj_client_;
        delete l_traj_client_;
    }

    //! Sends the command to start a given trajectory
    void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal, std::string arm)
    {
        // When to start the trajectory: 1s from now
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        if (arm.compare("right")==0)
            r_traj_client_->sendGoal(goal);
        else
            l_traj_client_->sendGoal(goal);
    }

    //! Generates a simple trajectory with two waypoints, used as an example
    /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
    pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory(std::string arm_name)
    {
        //our goal variable
        pr2_controllers_msgs::JointTrajectoryGoal goal;

        if (arm_name.compare("right")==0)
        {
            // First, the joint names, which apply to all waypoints
            goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
            goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
            goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
            goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
            goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
            goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
            goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

            // We will have 1 waypoints in this goal trajectory
            goal.trajectory.points.resize(1);

            // First trajectory point
            // Positions
            int ind = 0;
            goal.trajectory.points[ind].positions.resize(7);
            goal.trajectory.points[ind].positions[0] = -1.43;
            goal.trajectory.points[ind].positions[1] = -0.35;
            goal.trajectory.points[ind].positions[2] = -1.51;
            goal.trajectory.points[ind].positions[3] = -1.47;
            goal.trajectory.points[ind].positions[4] = 5.05;
            goal.trajectory.points[ind].positions[5] = -1.65;
            goal.trajectory.points[ind].positions[6] = -3.14;
            // Velocities
            goal.trajectory.points[ind].velocities.resize(7);
            for (size_t j = 0; j < 7; ++j)
            {
                goal.trajectory.points[ind].velocities[j] = 0.0;
            }
            // To be reached 1 second after starting along the trajectory
            goal.trajectory.points[ind].time_from_start = ros::Duration(5.0);

            //we are done; return the goal
            return goal;
        }
        else
        {
            //our goal variable
            pr2_controllers_msgs::JointTrajectoryGoal goal;

            // First, the joint names, which apply to all waypoints
            goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
            goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
            goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
            goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
            goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
            goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
            goal.trajectory.joint_names.push_back("l_wrist_roll_joint");

            // We will have two waypoints in this goal trajectory
            goal.trajectory.points.resize(1);

            // First trajectory point
            // Positions
            int ind = 0;
            goal.trajectory.points[ind].positions.resize(7);
            goal.trajectory.points[ind].positions[0] = 1.43;
            goal.trajectory.points[ind].positions[1] = -0.35;
            goal.trajectory.points[ind].positions[2] = 1.51;
            goal.trajectory.points[ind].positions[3] = -1.47;
            goal.trajectory.points[ind].positions[4] = 1.23;
            goal.trajectory.points[ind].positions[5] = -1.65;
            goal.trajectory.points[ind].positions[6] = -3.14;
            // Velocities
            goal.trajectory.points[ind].velocities.resize(7);
            for (size_t j = 0; j < 7; ++j)
            {
                goal.trajectory.points[ind].velocities[j] = 0.0;
            }
            // To be reached 1 second after starting along the trajectory
            goal.trajectory.points[ind].time_from_start = ros::Duration(5.0);

            //we are done; return the goal
            return goal;
        }
    }

    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState(std::string arm_name)
    {
        if (arm_name.compare("right")==0)
            return r_traj_client_->getState();
        else
            return l_traj_client_->getState();
    }

};

#endif // ROBOTARM_H
