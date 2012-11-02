#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
//#include <std_msgs/String.h>
#include <position_tracker/Position.h>
#include <path_navigator/Waypoints.h>
#include <map_loader/Node.h>
#include <map_loader/Edge.h>
#include <map_loader/GraphMap.h>
#include <path_generator/generatePath.h>
#include <path_generator/availableNextHops.h>
//#include <path_generator/dijkstras.h>
#include <math.h>
//#include <limits.h>

using namespace std;
using namespace map_loader;

ros::Publisher waypo_pub;
ros::Subscriber gmap_sub; 
vector<Node> nodes;
vector<Edge> edges;
vector<Node> waypoints;
GraphMap gmap;
ros::ServiceServer service, service2;
int map_loaded = 0;

void GraphMapCallback(const map_loader::GraphMapConstPtr& msg);
bool gen_path(path_generator::generatePath::Request &req, path_generator::generatePath::Response &res);
void RandomWalk(map_loader::GraphMap gmap, position_tracker::Position init_pos, path_navigator::Waypoints *w, int no_steps);


//------------- Dijkstra related code ----------
vector<vector<int> > neighbors;
vector<int> previous;
vector<int> erased;
int erased_num;

void Dijkstras(map_loader::GraphMap gmap, position_tracker::Position init_pos, position_tracker::Position dst_pos, path_navigator::Waypoints *w);
vector<int> AdjacentRemainingNodes(map_loader::Node node);
int ExtractSmallest(vector<map_loader::Node>& nodes);
int Distance(map_loader::Node node1, map_loader::Node node2);
bool Contains(vector<map_loader::Node>& nodes, map_loader::Node node);
void PrintLoadShortestRouteTo(map_loader::Node destination);
extern void DijkstrasTest();
void calculateNeighbors();
// these two not needed
//vector<Edge> AdjacentEdges(vector<map_loader::Edge>& Edges, map_loader::Node node);
//void RemoveEdge(vector<map_loader::Edge>& Edges, map_loader::Edge edge);
bool Connects(Edge edge, Node node1, Node node2);

bool get_available_next_hops(path_generator::availableNextHops::Request &req, path_generator::availableNextHops::Response &res);
int findGraphNode(position_tracker::Position p);



