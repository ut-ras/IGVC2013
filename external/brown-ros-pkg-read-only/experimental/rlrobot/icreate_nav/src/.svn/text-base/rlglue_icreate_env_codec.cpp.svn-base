/**
This is a copy of RL_client_environment for the irobot_createa couple
of specific calls filled in.  To customize the codec, just
look for places where default calls are happening, like:
env_init, env_start, env_step, etc, etc.. and then write a bit of code to
emulate the same behavior using whatever project you are hooking into.


All of the inserted code is commented.
All of the previous code in here has been commented out like:*/

/* CUT-FOR-CUSTOMIZATION: env_init();*/


#include <assert.h> /* assert  */
#include <unistd.h> /* sleep   */
#include <stdio.h>  /* fprintf */
#include <stdlib.h> /* calloc, getenv, exit */
#include <string.h> /* strlen */ /* I'm sorry about using strlen. */

#include <ctype.h> /* isdigit */
#include <netdb.h> /* gethostbyname */
#include <arpa/inet.h> /* inet_ntoa */

//#include <rlglue/Environment_common.h>
//#include <rlglue/network/RL_network.h>

/* Our project specific include */
#include "../include/icreate_glue.h"
/* Include the utility methods*/
//#include <rlglue/utils/C/RLStruct_util.h>

/* State variable for TheGame that is not exposed with function calls */

//using namespace std;


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

double GOAL_LOWX=0;
double GOAL_HIGHX=1;
double GOAL_LOWY=3;
double GOAL_HIGHY=4;
double GOAL_LOWYAW=1;
double GOAL_HIGHYAW=1.7;


static const char* kUnknownMessage = "Unknown Message: %s\n";

static action_t theAction                 = {0};
static rlBuffer theBuffer                 = {0};
static char* theInMessage = 0;
static unsigned int theInMessageCapacity = 0;

/*Added as a global variable because now we must allocate and fill up the data structures instead of the environment.*/
static observation_t globalObservation            = {0};

static void onEnvInit(int theConnection) {
  /* CUT-FOR-CUSTOMIZATION:  char* theTaskSpec = 0;*/
  unsigned int theTaskSpecLength = 0;
  unsigned int offset = 0;
  
  static char* theTaskSpec="VERSION RL-Glue-3.0 PROBLEMTYPE episodic DISCOUNTFACTOR 1 OBSERVATIONS DOUBLES (2 -2.0 10.0) (-2.0 2.0) ACTIONS INTS (0 3)  REWARDS (-1.0 10.0) EXTRA iRobotCreate (C/C++) by Sarah Osentoski.";
    
  if (theTaskSpec!=NULL) {
    theTaskSpecLength = strlen(theTaskSpec);//.length();
    printf("TaskSpecLength %d\n", theTaskSpecLength);
  }
  
  /* Prepare the buffer for sending data back to the server */
  rlBufferClear(&theBuffer);
  offset = rlBufferWrite(&theBuffer, offset, &theTaskSpecLength, 1, sizeof(int));
  if (theTaskSpecLength > 0) {
    offset = rlBufferWrite(&theBuffer, offset, theTaskSpec, theTaskSpecLength, sizeof(char));
  }
}

static void onEnvStart(int theConnection, ros::Rate loop_rate) {
  /* CUT-FOR-CUSTOMIZATION: observation_t globalObservation = {0}; */
  static observation_t this_observation;
  
  /*alocate space to store the observation*/
  /*TODO needs to be changed to take a variable number of vars*/
  allocateRLStruct(&this_observation, 0, 3, 0);
   
  unsigned int offset = 0;
  
  /* Call the start environment and get new variables */
  /*TODO also needs to be changed to not be variable specific*/
  position_tracker::Position ros_observation=start_environment(loop_rate);
  this_observation.doubleArray[0]=ros_observation.x;
  this_observation.doubleArray[1]=ros_observation.y;
  this_observation.doubleArray[2]=ros_observation.theta;


  
  /*send to RL-glue*/

  __RL_CHECK_STRUCT(&this_observation)
    rlBufferClear(&theBuffer);
  offset = rlCopyADTToBuffer(&this_observation, &theBuffer, offset);
  printf("onEnvStart \n");
}

static void onEnvStep(int theConnection, ros::Rate loop_rate, ros::ServiceClient client) {
  static reward_observation_terminal_t ro;// = {0};
  unsigned int offset = 0;
  /* Create an integer variable to hold the action from the agent*/
  int theIntAction=0;
  
  printf("onEnvStep\n");
  
  offset = rlCopyBufferToADT(&theBuffer, offset, &theAction);
  __RL_CHECK_STRUCT(&theAction);
  
  assert(theAction.numInts==1);
  assert(theAction.intArray[0]>=0);
  assert(theAction.intArray[0]<4);
  
  
  /*I know to only expect 1 integer action*/
  theIntAction=theAction.intArray[0];
  
  /*This is our hook into the robot */
  printf("about to go in\n");
  position_tracker::Position tempobs=take_step(theIntAction, loop_rate, client);
  printf("finished\n");
  ro=convertObservation(tempobs);
  ro.reward=calculateReward(tempobs);
  ro.terminal=checkTerminal(tempobs);


  
  __RL_CHECK_STRUCT(ro.observation)
    rlBufferClear(&theBuffer);
  
  offset = 0;
  offset = rlBufferWrite(&theBuffer, offset, &ro.terminal, 1, sizeof(int));
  offset = rlBufferWrite(&theBuffer, offset, &ro.reward, 1, sizeof(double));
  offset = rlCopyADTToBuffer(ro.observation, &theBuffer, offset);
}

static void onEnvCleanup(int theConnection) {
  /*No game specific cleanup to do*/
  /* CUT-FOR-CUSTOMIZATION: env_cleanup();*/
  
  rlBufferClear(&theBuffer);
  
  /* Clean up globalObservation global we created*/
  clearRLStruct(&globalObservation);

  clearRLStruct(&theAction);
  
  /*It's ok to free null pointers, so this is safe */
  free( theInMessage);
  theInMessage = 0;
  theInMessageCapacity = 0;
}


static void onEnvMessage(int theConnection) {
  unsigned int inMessageLength = 0;
  unsigned int outMessageLength = 0;
  char *inMessage=0;
  /*We set this to a string constant instead of null*/
  char *outMessage="sample custom codec integration has no messages!";
  unsigned int offset = 0;

  offset = 0;
  offset = rlBufferRead(&theBuffer, offset, &inMessageLength, 1, sizeof(int));
  if (inMessageLength >= theInMessageCapacity) {
    inMessage = (char*)calloc(inMessageLength+1, sizeof(char));
    free(theInMessage);

    theInMessage = inMessage;
    theInMessageCapacity = inMessageLength;
  }


  if (inMessageLength > 0) {
    offset = rlBufferRead(&theBuffer, offset, theInMessage, inMessageLength, sizeof(char));
  }
  /*Make sure to null terminate the string */
  theInMessage[inMessageLength]='\0';
  
  /* CUT-FOR-CUSTOMIZATION: outMessage = env_message(theInMessage);*/
  
  if (outMessage != NULL) {
    outMessageLength = strlen(outMessage);
  }
  
  
  /* we want to start sending, so we're going to reset the offset to 0 so we write the the beginning of the buffer */
  rlBufferClear(&theBuffer);
  offset = 0;
  offset = rlBufferWrite(&theBuffer, offset, &outMessageLength, 1, sizeof(int));
  if (outMessageLength > 0) {
    offset = rlBufferWrite(&theBuffer, offset, outMessage, outMessageLength, sizeof(char));
  }
}

void runEnvironmentEventLoop(int theConnection, ros::Rate loop_rate, ros::ServiceClient client) {
  int envState = 0;
  printf("In event loop \n");
 
  do {
    rlBufferClear(&theBuffer);
    rlRecvBufferData(theConnection, &theBuffer, &envState);
    
    printf("envstate %d\n", envState);
    printf("envInit %d, EnvStart %d, kEnvStep %d, kEnvCleanup %d, \n",kEnvInit, kEnvStart, kEnvStep, kEnvCleanup);
    switch(envState) {
    case kEnvInit:
      onEnvInit(theConnection);
      break;

    case kEnvStart:
      onEnvStart(theConnection, loop_rate);
      break;

    case kEnvStep:
      onEnvStep(theConnection, loop_rate, client);
      break;

    case kEnvCleanup:
      onEnvCleanup(theConnection);
      break;

    case kEnvMessage:
      onEnvMessage(theConnection);
      break;

    case kRLTerm:
      break;

    default:
      fprintf(stderr, kUnknownMessage, envState);
      exit(0);
      break;
    };

    rlSendBufferData(theConnection, &theBuffer, envState);
  } while (envState != kRLTerm);
}

/*This used to be the main method, I've renamed it and cut a bunch of stuff out*/
int setup_rlglue_network() {
        int theConnection = 0;
	struct hostent *host_ent;

        char *host = kLocalHost;
        short port = kDefaultPort;
	char *envptr =0;

	host = getenv("RLGLUE_HOST");
	if (host == 0) {
	  host = kLocalHost;
	}
	
	envptr = getenv("RLGLUE_PORT");  
	if (envptr != 0) {
	  port = strtol(envptr, 0, 10);
	  if (port == 0) {
	    port = kDefaultPort;
	  }
	}
	if (isalpha(host[0])) {
	  /*This method is apparently deprecated, we should update at some point*/
	  host_ent = gethostbyname(host); 
	  if(host_ent==0){
	    fprintf(stderr,"Couldn't find IP address for host: %s\n",host);
	    exit(55);
	  }
	  host = inet_ntoa(*(struct in_addr*)host_ent->h_addr_list[0]);
	}
	
	//  	fprintf(stdout, "RL-Glue C Environment Codec Version %s, Build %s\n\tConnecting to host=%s on port=%d...\n", VERSION,__rlglue_get_codec_svn_version(),host, port);
	//fflush(stdout);
	
	
	
	
        printf("RL-Glue sample env custom codec integration.\n");

        /* Allocate what should be plenty of space for the buffer - it will dynamically resize if it is too small */
        rlBufferCreate(&theBuffer, 4096);
	
	printf("RL-glue about to wait for connection \n");
        theConnection = rlWaitForConnection(host, port, kRetryTimeout);

        printf("\tSample custom env codec :: Connected\n");
        rlBufferClear(&theBuffer);
        rlSendBufferData(theConnection, &theBuffer, kEnvironmentConnection);

        return theConnection;
}

void teardown_rlglue_network(int theConnection){
        rlClose(theConnection);
        rlBufferDestroy(&theBuffer);
}


/****** Helper Functions ****/

reward_observation_terminal_t  convertObservation(position_tracker::Position observation){
  static observation_t this_observation;
  static reward_observation_terminal_t this_reward_observation;

  allocateRLStruct(&this_observation, 0, 3, 0);
  this_reward_observation.observation=&this_observation;

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
      //     cout << "terminal position" << endl;
      return true;
    }
  else
    return false;
  
}
