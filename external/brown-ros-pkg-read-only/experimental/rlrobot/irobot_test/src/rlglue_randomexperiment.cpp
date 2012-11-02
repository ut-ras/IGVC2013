#include <stdio.h>    // for sprintf
#include <math.h>     // for sqrt

#include <iostream>   // for cout
#include <fstream>   
#include <string>

#include <rlglue/RL_glue.h> /* RL_ function prototypes and RL-Glue types */

using namespace std; 


typedef struct {
	double mean;
	double standard_dev;
} evaluation_point_t;

evaluation_point_t *evaluate_agent();
void single_evaluation();
void print_score(int afterEpisodes, evaluation_point_t *the_score);
void save_results_csv(evaluation_point_t *the_score[], char *fileName);
void offline_demo();

int main(int argc, char *argv[]) {

  cout << "Starting random walk " << endl
       << "----------------------------" << endl
       << endl << endl; 
  
  cout << "After Episode" << endl
       << "Mean Return\tStandard Deviation" << endl
       << "-----------------------------------------------------------------"
       << endl; 
  
  RL_init();
  offline_demo();
  
  cout << endl << 
    "Calling RL_cleanup and RL_init to clear the agent's memory..." << endl; 
  
  RL_cleanup();

  printf("\nProgram Complete.\n");
  
  return 0;
}



/*
  This function will freeze the agent's policy
  and test it after every 25 episodes.
*/
void offline_demo(){

  int j=0, k=0;
  // while(1){  
  for(j=0;j<200;j++){
    RL_episode(5000);
    cout <<"Switching to Random" << endl;
    RL_agent_message("switchToRandom");
    for(k=0;k<200;k++){
      cout << "Rstep "<<k<<endl;
      RL_episode(2);
    }
    cout <<"Switching to Learner" << endl;
    RL_agent_message("switchToLearner");
    cout << "Step "<< j <<endl;
    
  }

  
}

