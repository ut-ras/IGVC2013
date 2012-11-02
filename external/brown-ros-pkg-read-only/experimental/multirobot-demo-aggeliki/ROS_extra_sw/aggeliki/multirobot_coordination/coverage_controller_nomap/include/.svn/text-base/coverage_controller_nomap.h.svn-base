#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <wifi_comm/WiFiNeighboursList.h>
#include <position_tracker/Position.h>
#include "std_msgs/String.h"

#include <wifi_comm/wifi_comm_lib.h>

using namespace std;
typedef map<string, position_tracker::Position> mapType;

WiFiComm * myComm;
ros::NodeHandle * n;


// reading a text file
#include <string>
using namespace std;

int main () {
  string line;
  ifstream myfile ("example.txt");
  if (myfile.is_open())
  {
    while ( myfile.good() )
    {
      getline (myfile,line);
      cout << line << endl;
    }
    myfile.close();
  }

  else cout << "Unable to open file";

  return 0;
}



char *my_ip = "192.168.0.6";
position_tracker::Position cur_pos;

std::vector<ros::Subscriber> subs;

std::map<string, position_tracker::Position > nn_pos; //the last position message from  neighboring IPs

void nextMoveController();


