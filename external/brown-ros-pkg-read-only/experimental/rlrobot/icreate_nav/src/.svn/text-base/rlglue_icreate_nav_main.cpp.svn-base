#include <cassert>
#include <string>
#include <iostream>
#include <sstream>
#include <math.h>
#include <cmath>
#include <stdlib.h>

#include "icreate_glue.h"
//#include <ros/ros.h>
//#include <discreteMove_0_0_1/Act.h>
//#include <position_tracker/Position.h>

// env_ function prototypes types 
#include <rlglue/Environment_common.h>	  

// helpful functions for allocating structs and cleaning them up 
#include <rlglue/utils/C/RLStruct_util.h> 

using namespace std;


/*Global ROS Variables */
position_tracker::Position current_observation;

/* GLOBAL VARIABLES FOR RL-GLUE methods (global for convenience) */  
/* TO DO: Move to a Reward function file*/
//#define GOAL_LOWX 0
//#define GOAL_HIGHX 1
//#define GOAL_LOWY 0
//#define GOAL_HIGHY 1

//double GOAL_LOWX=3.3;
//double GOAL_HIGHX=3.6;
//double GOAL_LOWY=0;
//double GOAL_HIGHY=.5;
//double GOAL_LOWYAW=2;
//double GOAL_HIGHYAW=3.5;

double GOAL_LOWX=3.5;
double GOAL_HIGHX=4;
double GOAL_LOWY=0;
double GOAL_HIGHY=1;
double GOAL_LOWYAW=-1.2;
double GOAL_HIGHYAW=1.7;


/*****************************

	Ros Methods 
	
*******************************/

//Actions:
#define LEFT 1
#define FORWARD 2
#define RIGHT 3

bool call(ros::ServiceClient client, int action){
  discreteMove_0_0_1::Act act;
  act.request.action=action;
  if (client.call(act)){
    //success!
  }
  else{
    //failure!
    return act.response.done;
  }
}
bool left(ros::ServiceClient client){
  return call(client,LEFT);
}

bool right(ros::ServiceClient client){
  return call(client,RIGHT);
}

bool forward(ros::ServiceClient client){
  return call(client, FORWARD);
}

void observation_handler(const position_tracker::PositionConstPtr& msg){  
  //  cout << "here" << endl;
  
  current_observation.x=msg->x;
  current_observation.y=msg->y;
  current_observation.theta=msg->theta;
  
}


/*****************************

	RL-Glue Methods 
	
*******************************/

position_tracker::Position start_environment()
{
  /* see where the robot is.  And then send that position*/
  
  ros::spinOnce();
  loop_rate.sleep();
  
  position_tracker::Position observation= getObservation();
  
  cout << observation.x  << endl;  
  return observation;
  
  
  this_observation.doubleArray[0]=observation.x;
  this_observation.doubleArray[1]=observation.y;
  this_observation.doubleArray[2]=observation.theta;

  cout << "we are started" << endl;

  return &this_observation;
}


position_tracker::Position take_step(int action)
{  

  cout << "taking action " <<action << endl;
  /*Execute the Action */
  if(action==LEFT){
    left(client);
  }
  else if(action==FORWARD){
    forward(client);
  }
  else{
    right(client);
  }
  
  cout << "took an action" << endl;
  ros::spinOnce();
  loop_rate.sleep();
   
  //check to make sure this observation is ok
  
  position_tracker::Position observation= getObservation();
  return observation;
  /*
  cout << "going to convert"<< endl;
  convertObservation(observation);
  cout << "converted"<<endl;

  this_reward_observation.reward= calculateReward(observation);
  this_reward_observation.terminal=checkTerminal(observation);
  
    cout << "we took a step" << endl;
    return &this_reward_observation;
  */
}


int main(int argc, char** argv)
{

  //Ros variables
  bool rosinitialized=false;
  ros::NodeHandle n;
  ros::ServiceClient client;
  ros::Subscriber observations;
  ros::Rate loop_rate(10);
  
  
  srand((unsigned)time(0));
  //Initialize Ros
  ros::init(argc, argv, "irobot_test");
     
  //Subscribe to service discreteMove_0_0_1
  client = n.serviceClient<discreteMove_0_0_1::Act>("act");

  //Subscribe to message position with buffer of 100 function: observation_handler
  observations=n.subscribe("position", 100, observation_handler);

  cout << "Oh my we are testing" << endl;
  
  int theConnection=setup_rlglue_network();
  runEnvirionmentEventLoop(theConnection);
  
  cout << "Finished" << endl;
  teardown_rlglue_network(theConnection);

  
  return 1;
 

}

  
/****************************


        Helper Functions


*****************************/

  
position_tracker::Position getObservation(){
  cout << "get observation "<< endl;

  position_tracker::Position  obj=current_observation;
  return obj;
}
  
reward_observation_terminal_t  convertObservation(position_tracker::Position observation){
  static reward_observation_terminal_t this_reward_observation;
  allocateRLStruct(&this_reward_observation, 0, 3, 0);
  this_reward_observation.observation->doubleArray[0]=observation.x;
  this_reward_observation.observation->doubleArray[1]=observation.y;
  this_reward_observation.observation->doubleArray[2]=observation.theta;
  return this_reward_observation;
  
}

double calculateReward(position_tracker::Position observation){  
  double distance;
  if(observation.x>=GOAL_LOWX && observation.x<=GOAL_HIGHX && observation.y>=GOAL_LOWY && observation.y <= GOAL_HIGHY && fabs(observation.theta) <=GOAL_HIGHYAW && fabs(observation.theta)>=GOAL_LOWYAW)
    return 10;
  else
    {
      distance=sqrt(pow((GOAL_HIGHX-observation.x), 2)+pow((GOAL_HIGHY-observation.y), 2));
      return -1*distance;
    }
  
}


bool checkTerminal(position_tracker::Position observation){  
  if(observation.x>=GOAL_LOWX && observation.x<=GOAL_HIGHX && observation.y>=GOAL_LOWY && observation.y <= GOAL_HIGHY && fabs(observation.theta) <=GOAL_HIGHYAW && fabs(observation.theta)>=GOAL_LOWYAW)
    {
      cout << "terminal position" << endl;
      return true;
    }
  else
    return false;
  
}


