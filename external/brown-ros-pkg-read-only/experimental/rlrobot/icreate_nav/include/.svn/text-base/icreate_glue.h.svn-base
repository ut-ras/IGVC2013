//ROS Includes:
#include <ros/ros.h>
#include <discreteMove_0_0_1/Act.h>
#include <position_tracker/Position.h>

//RL-Glue Includes:
#include <rlglue/Environment_common.h>
#include <rlglue/network/RL_network.h>
#include <rlglue/utils/C/RLStruct_util.h>



#ifndef ICREATE_GLUE_H
#define ICREATE_GLUE_H


/* Implemented in icreate_nav.c*/
position_tracker::Position start_environment(ros::Rate loop_rate);
position_tracker::Position take_step(int action, ros::Rate loop_rate, ros::ServiceClient);
position_tracker::Position getObservation();

/*Implemented in rlglue_icreate_env_codec*/
reward_observation_terminal_t convertObservation(position_tracker::Position observation);
double calculateReward(position_tracker::Position observation);
bool checkTerminal(position_tracker::Position observation);


int setup_rlglue_network();
void teardown_rlglue_network(int theConnection);
void runEnvironmentEventLoop(int theConnection, ros::Rate loop_rate, ros::ServiceClient client);

#endif
