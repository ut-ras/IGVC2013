#include "ros/ros.h"
#include "irobot_test/LocalizableObject.h"
#include "irobot_test/Locations.h"
#include "discreteMove_0_0_1/Act.h"

#include <iostream>
#include <cstdlib>

//actions
#define LEFT 1
#define FORWARD 2
#define RIGHT 3

using namespace std;

//global (ugh) observations
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

int randint(){
  int lowest=1, highest=3;
  int range=(highest-lowest)+1;
  //int random_integer=(rand()%range)+1;
  //int random_integer = (int)((double)rand()/(double)RAND_MAX * range *lowest+.5);
  
  int random_integer=lowest+(int)((range*rand()/(RAND_MAX))+.5);
  return random_integer;
}

void main(int argc, char* argv[])
{
  //setup ros stuff
  srand((unsigned)time(0));
  ros::init(argc, argv, "randomWalker");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<discreteMove_0_0_1::Act>("act");
  ros::Subscriber topobservations=n.subscribe("topDownLocations", 100,observation_handler);
  
  ros::Rate loop_rate(10);
  irobot_test::Locations observation;


  int numRanSteps=50;

  for (int i=0; i<numRanSteps; i++)
    {
      srand((unsigned)time(0));
      
      //generate a random action;
      int action=randint();
      cout << "Action is " << action << endl;

      //execute the random action;
      if(action==LEFT){
	left(client);
      }
      else if(action==RIGHT){
	right(client);
	}
      else{
	forward(client);
      }
      

      //make an observation
      ros::spinOnce();
      loop_rate.sleep();
      observation=current_observation;
      
      //spit out what the observation is.
     
      cout << "Number of objects is " <<observation.objectlist.size()<<endl;
      
      cout << "First object type is "<< observation.objectlist.at(0).objecttype << " with id " << observation.objectlist.at(0).objectid << " at positin x " << observation.objectlist.at(0).posx << " y " << observation.objectlist.at(0).posy << " yaw " << observation.objectlist.at(0).yaw <<endl;
    }
  
  
}
