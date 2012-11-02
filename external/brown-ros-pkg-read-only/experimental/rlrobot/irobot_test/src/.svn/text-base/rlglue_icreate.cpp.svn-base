#include <cassert>
#include <string>
#include <iostream>
#include <sstream>

#include <stdlib.h>

#include <ros/ros.h>
#include <irobot_test/LocalizableObject.h>
#include <irobot_test/Locations.h>
#include <discreteMove_0_0_1/Act.h>
#include "../include/localizableobject.h"
#include "../include/objecttype.h"

// env_ function prototypes types 
#include <rlglue/Environment_common.h>	  

// helpful functions for allocating structs and cleaning them up 
#include <rlglue/utils/C/RLStruct_util.h> 


using namespace std;
using namespace rlab;

//FUNCITON DEFS

irobot_test::LocalizableObject getObservation();
void  convertObservation(irobot_test::LocalizableObject observation);
double calculateReward(irobot_test::LocalizableObject observation);
bool checkTerminal(irobot_test::LocalizableObject observation);

/* GLOBAL VARIABLES FOR RL-GLUE methods (global for convenience) */  

//#define GOAL_LOWX 0
//#define GOAL_HIGHX 1
//#define GOAL_LOWY 0
//#define GOAL_HIGHY 1

double GOAL_LOWX=3.3;
double GOAL_HIGHX=3.6;
double GOAL_LOWY=0;
double GOAL_HIGHY=.5;
double GOAL_LOWYAW=2;
double GOAL_HIGHYAW=3.5;


static observation_t this_observation;
static reward_observation_terminal_t this_reward_observation;


static string task_spec_string = 
"VERSION RL-Glue-3.0 PROBLEMTYPE episodic \
<<<<<<< .mine
DISCOUNTFACTOR 1 OBSERVATIONS DOUBLES (2 0 5) (-2 2) \
ACTIONS INTS (1 3)  REWARDS (-1.0 10.0) \
EXTRA iRobotCreate (C/C++) by Sarah Osentoski.";


/*****************************

	Ros Methods 
	
*******************************/

//Actions:
#define LEFT 1
#define FORWARD 2
#define RIGHT 3


//ROS Globals: make these a class later
bool rosinitialized=false;
ros::NodeHandle *n;
ros::ServiceClient client;
ros::Subscriber topobservations;
ros::Rate loop_rate(10);

irobot_test::Locations current_observation;


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

void observation_handler(const irobot_test::LocationsConstPtr & msg){  
  cout << "here" << endl;

  if (msg->objectlist.size()==0)
    return;
  
  current_observation.objectlist.assign(msg->objectlist.begin(), msg->objectlist.end());
  
}

int getNumObjects(){
  return (int)current_observation.objectlist.size();
}

irobot_test::LocalizableObject getObject(int i)
{
  return current_observation.objectlist.at(i);
}




/*****************************

	RL-Glue Methods 
	
*******************************/

const char* env_init(){
  int argc=0;
  char **argv;
  //Allocate and set up the ROS stuff
   srand((unsigned)time(0));

  if( ~rosinitialized){
    ros::init(argc, argv, "randomWalker");
  
    n=new ros::NodeHandle;
    
    client = n->serviceClient<discreteMove_0_0_1::Act>("act");
    topobservations=n->subscribe("topDownLocations", 100,observation_handler);
  }

  cout << "Oh my we are testing" << endl;
  allocateRLStruct(&this_observation, 0, 3, 0);  
  this_reward_observation.observation=&this_observation;//this might be bad

  /*  irobot_test::LocalizableObject observation;
  //temporary stuff to test while top down loc is *not* running
  LocalizableObject topdownobjects;

  observation.objecttype=topdownobjects.OBJECT_TYPE_SMURV;
  observation.objectid=1;
  observation.posx=1.0;
  observation.posy=1.0;
  observation.yaw=.5;
  current_observation.objectlist.push_back(observation);
  
  
  allocateRLStruct(&this_observation, 0, 3, 0);

  this_reward_observation.observation=&this_observation;
  this_reward_observation.reward=0;
  this_reward_observation.terminal=0;
  */
  return task_spec_string.c_str();
}


const observation_t *env_start()
{
  /* see where the robot is.  And then send that position*/

  
  ros::spinOnce();
  loop_rate.sleep();
  
  
  irobot_test::LocalizableObject observation= getObservation();
  
  
  cout << observation.posx  << endl;  
  this_observation.doubleArray[0]=observation.posx;
  this_observation.doubleArray[1]=observation.posy;
  this_observation.doubleArray[2]=observation.yaw;

  cout << "we are started" << endl;

  return &this_observation;
}


const reward_observation_terminal_t *env_step(const action_t *this_action)
{  
  /*Check to make sure action is valid*/
  assert(this_action->numInts==1);
  assert(this_action->intArray[0]>=0);
  assert(this_action->intArray[0]<4);
  
  int action= this_action->intArray[0];
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
  
  irobot_test::LocalizableObject observation= getObservation();
  cout << "going to convert"<< endl;
  convertObservation(observation);
  cout << "converted"<<endl;

  this_reward_observation.reward= calculateReward(observation);
  this_reward_observation.terminal=checkTerminal(observation);
  
    cout << "we took a step" << endl;
    return &this_reward_observation;
    
}

void env_cleanup()
{
  clearRLStruct(&this_observation);
  //  clearRLStruct(&this_reward_observation);  

}

const char* env_message(const char* _inMessage){

}


  
/****************************


        Helper Functions


*****************************/

  
irobot_test::LocalizableObject getObservation(){
  cout << "get observation "<< endl;

  irobot_test::Locations obs;
  irobot_test::LocalizableObject obj;
  int finished=0;
  cout << "before loop" << endl;

  int loopnum=0;
  while(finished==0){
    obs=current_observation;
    /*there shouldn't ever be no objects*/
    if( obs.objectlist.size()<=0)
      while(obs.objectlist.size()<=0){
	ros::spinOnce();
	loop_rate.sleep();
	obs=current_observation;
      }
    cout << "there were objects " << obs.objectlist.size() << endl;
    

    int foundloc=-1;
    for(int i=0; i<(int)obs.objectlist.size(); i++)
      {
	
	obj=obs.objectlist.at(i);
	

	//obj=current_observation(i);
	
	cout << "obj type " <<obj.objectid  << endl;
	if(obj.objecttype==OBJ_TYPE_SMURV){
	  foundloc=i;
	  i=obs.objectlist.size();
	  finished=1;
	}
      }
    
    cout << "foundloc: "<< foundloc<<endl;

    if(foundloc<0){
      ros::spinOnce();
      loop_rate.sleep();

      loopnum++;
    }  
    cout<< "loopnum " << loopnum;
    if(loopnum>100)
      abort();
  }



  return obj;
}
  
  void  convertObservation(irobot_test::LocalizableObject observation){
    this_reward_observation.observation->doubleArray[0]=observation.posx;
    this_reward_observation.observation->doubleArray[1]=observation.posy;
    this_reward_observation.observation->doubleArray[2]=observation.yaw;
    
  }

double calculateReward(irobot_test::LocalizableObject observation){  
  if(observation.posx>=GOAL_LOWX && observation.posx<=GOAL_HIGHX && observation.posy>=GOAL_LOWY && observation.posy <= GOAL_HIGHY && fabs(observation.yaw) <=GOAL_HIGHYAW && fabs(observation.yaw)>=GOAL_LOWYAW)
    return 10;
  else
    return -1.0;
  
}


bool checkTerminal(irobot_test::LocalizableObject observation){  
  if(observation.posx>=GOAL_LOWX && observation.posx<=GOAL_HIGHX && observation.posy>=GOAL_LOWY && observation.posy <= GOAL_HIGHY && fabs(observation.yaw) <=GOAL_HIGHYAW && fabs(observation.yaw)>=GOAL_LOWYAW)
    {
      cout << "terminal position" << endl;
      return true;
    }
  else
    return false;
  
}


