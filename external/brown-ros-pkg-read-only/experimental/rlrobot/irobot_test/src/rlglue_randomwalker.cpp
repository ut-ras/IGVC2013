#include <string>
#include <cassert>
#include <iostream>
#include <fstream>

#include <rlglue/Agent_common.h>	  

// helpful functions for allocating structs and cleaning them up 
#include <rlglue/utils/C/RLStruct_util.h> 
#include <rlglue/utils/C/TaskSpec_Parser.h> /* task spec parser */


using namespace std;

action_t this_action;
int randInRange(int max);
int numActions;

void agent_init(const char* task_spec)
{
  /*Struct to hold the parsed task spec*/
  srand((unsigned)time(0));
  taskspec_t *ts= new taskspec_t; 
  int decode_result = decode_taskspec( ts, task_spec );
  if(decode_result!=0){
    cerr << "Could not decode task spec, code: " << decode_result
         << "for task spec: " << task_spec << endl; 
    exit(1);
  }
  assert(getNumIntAct(ts)==1);
  assert(getNumDoubleAct(ts)==0);
  assert(isIntActMax_special(ts,0)==0);
  assert(isIntActMin_special(ts,0)==0);
  
  numActions=getIntActMax(ts,0)+1;
  
  free_taskspec_struct(ts); // Make the taskspec struct a "blank slate" 

  delete ts; // Free the structure itself 

  allocateRLStruct(&this_action, 1, 0, 0);

}

const action_t *agent_step(double reward, const observation_t *this_observation){

  int act= randInRange(numActions-1); 
  cout << "Action is " << act << endl;
  this_action.intArray[0]=act;
  return & this_action;
}

const action_t *agent_start(const observation_t *this_observation)
{
  
  return agent_step(0, this_observation);
  
}

void agent_end(double reward){
  
}

void agent_cleanup(){

}


const char* agent_message(const char* _inMessage){

  string buffer;
  string inMessage = _inMessage;
  

  return "RandomAgent(C/C++) does not take messages because it is random.";
		

}



int randInRange(int max){
  //double r, x;
  // r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
  // x = (r * (max+1));
  
  int lowest=1;
  int range=(max-lowest);

 int random_integer = lowest+(int)((double)rand()/(double)RAND_MAX * range *lowest+.5);
  
  return random_integer;
}
