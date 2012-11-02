#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <list>
#include <numeric>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <map_loader/Line.h>
#include <map_loader/Node.h>
#include <map_loader/Edge.h>
#include <map_loader/LineMap.h>
#include <map_loader/GraphMap.h>


using namespace std;

ros::Publisher lines_pub;
ros::Publisher graph_pub;
ros::Rate loop_rate(1);

map_loader::LineMap *lmap;
map_loader::GraphMap *gmap;

//void Tokenize(const string& str, vector<string>& tokens, const string& delimiters=" ");
void loadMap();
void error(char *msg);




