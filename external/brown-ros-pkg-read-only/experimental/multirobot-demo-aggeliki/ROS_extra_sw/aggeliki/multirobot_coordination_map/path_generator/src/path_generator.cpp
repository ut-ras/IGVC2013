#include "path_generator.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "path_generator");
     ros::NodeHandle n1, n2;
     waypo_pub = n1.advertise<path_navigator::Waypoints>("/waypoints", 1);
     gmap_sub = n2.subscribe("/graph_map", 1, GraphMapCallback);

     ros::spinOnce(); 

     service = n1.advertiseService("generate_path", gen_path);
     service2 = n1.advertiseService("available_next_hops", get_available_next_hops);

   /*  while(1)
     {
	//------------- JUST FOR testing ------------------
  	 ros::ServiceClient client = n1.serviceClient<path_generator::generatePath>("generate_path");
     path_generator::generatePath srv; 
	 path_navigator::Waypoints *w = new path_navigator::Waypoints();
	 w->waypoints = waypoints;           
     srv.request.type = 0;
	 //TESTING... set your cur_pos through code... (then checkout the waypoints topic for the results) -> play with different values of cur_pos and dst_pos
	 position_tracker::Position *init_pos = new position_tracker::Position();
     init_pos->x = 10;
	 init_pos->y = 12.7;

     srv.request.init_pos = *init_pos;
     //srv.request.init_pos = cur_pos;
	 position_tracker::Position *dst_pos = new position_tracker::Position();
	 dst_pos->x = 15;
	 dst_pos->y = 11;
     srv.request.dst_pos = *dst_pos;

	gen_path(srv.request, srv.response);
	//-----------------------------------------------------


     ros::spinOnce(); 
     //ros::spin();    
	} */

     ros::spin();
}

bool gen_path(path_generator::generatePath::Request &req, path_generator::generatePath::Response &res)
{
	path_navigator::Waypoints *w = NULL;
	if(req.type == 0) //generate dikstra's path
	{
		w = new path_navigator::Waypoints();
		Dijkstras(gmap, req.init_pos, req.dst_pos, w);
	}
	else if(req.type == 1)
	{
		w = new path_navigator::Waypoints();
		RandomWalk(gmap, req.init_pos, w, 10); //default: perform 10 steps before you stop
	}

    res.w = *w;

	return true;
}

bool get_available_next_hops(path_generator::availableNextHops::Request &req, path_generator::availableNextHops::Response &res)
{
	int cur_id = findGraphNode(req.cur_pos);
	
	vector<int> neigh_ids;
	vector<position_tracker::Position>::iterator it;
	for(it = req.neighbor_pos.begin(); it != req.neighbor_pos.end(); ++it)
	{
		neigh_ids.push_back(findGraphNode(*it));
	}

	vector<int> cur_neighbors = neighbors.at(cur_id);	

	//don't consider any next hop nodes occupied by a neighbor of mine
	vector<int>::iterator it3;
	vector<int> res_vec = cur_neighbors;
	for(it3 = neigh_ids.begin(); it3!= neigh_ids.end(); ++it3)
	{
		vector<int>::iterator it4 = find(res_vec.begin(), res_vec.end(), (*it3));	
		if(it4 != res_vec.end()) 
			res_vec.erase(it4);
	}

	vector<int>::iterator it2;
	for(it2 = res_vec.begin(); it2 != res_vec.end(); ++it2)
	{
		res.avail_next_hops.push_back(nodes.at(*it2).p);
	}
		
	return true;
}

int findGraphNode(position_tracker::Position p)
{
	vector<map_loader::Node>::iterator itl;
	double min_dist = 10000;
	int min_ind = -1;
        for(itl = nodes.begin(); itl != nodes.end(); ++itl)
	{
		double dist = sqrt(pow(p.x - (*itl).p.x, 2) + pow(p.y - (*itl).p.y, 2));			
		if(dist < min_dist)
		{
			min_dist = dist;
			min_ind = itl - nodes.begin();
		} 
	}
	return min_ind;
}

void GraphMapCallback(const map_loader::GraphMapConstPtr& msg)
{
	if(map_loaded) return;	

	//else, load the map
	nodes = (*msg).nodes;
	edges = (*msg).edges;
	gmap = (*msg);
	calculateNeighbors();
	map_loaded = 1;
}

void RandomWalk(map_loader::GraphMap gmap, position_tracker::Position init_pos, path_navigator::Waypoints *w, int no_steps)
{
	//find the graph node that is closest to the initial position init_pos
	map_loader::Node init_node;
	double min_dist = 1000000;
	int min_ind = -1;
    int cnt = 0;
    vector<map_loader::Node>::iterator it;
    for(it = nodes.begin(); it != nodes.end(); ++it)
	{
		double dist = sqrt(pow(init_pos.x - (*it).p.x, 2) + pow(init_pos.y - (*it).p.y, 2));
		if(dist <= min_dist)
		{
			min_dist = dist;
			min_ind = cnt;
		}
		cnt++;
	}
    init_node = nodes[min_ind]; 

	//generate the next 10 (no_steps) nodes based on random walk
	int tmp_ind = init_node.id;
	for(int i=1; i < no_steps; i++)
	{
		vector<int> nextnodes = neighbors.at(tmp_ind);
		int random_ind = rand()%nextnodes.size();
		w->waypoints.push_back(nodes[nextnodes.at(random_ind)]);
		tmp_ind = nextnodes.at(random_ind);
	}
}


/*void getRandomWalkNextPos(position_tracker::Position *p)
{
    if(goal_pos_x < 0 || goal_pos_y < 0)
	{
		//find closest point of graph-based map given the current pos
		double min_dist = 1000000;
		int min_ind = -1;
        int cnt = 0;
        vector<Node *>::iterator it;
        for(it = nodes.begin(); it != nodes.end(); ++it)
		{
			double dist = sqrt(pow(cur_pos.x - (*it)->p->x, 2) + pow(cur_pos.y - (*it)->p->y, 2));
			if(dist <=  min_dist)
			{
				min_dist = dist;
				min_ind = cnt;
			}
			cnt++;
		}
        p->x = nodes[min_ind]->p->x;
        p->y = nodes[min_ind]->p->y;
		goal_ind = min_ind;
    }
	else{  
		//select randomly the next goal based on the neighbors of the current goal
		vector<Node *> neighbors;        
		vector<Edge *>::iterator it;

        cout << "goal_ind = " << goal_ind << endl;

        for(it = edges.begin(); it != edges.end(); ++it)
		{
			if((*it)->node1->id == goal_ind)
			{
				neighbors.push_back((*it)->node2);
		        cout << "node1->id == goal_ind" << endl;

			}
			else if((*it)->node2->id == goal_ind)
			{
				neighbors.push_back((*it)->node1);
		        cout << "node2->id == goal_ind" << endl;
			}
		}

        int N = neighbors.size();
		int rind = rand() % N;


        p->x = neighbors[rind]->p->x;
        p->y = neighbors[rind]->p->y;
		goal_ind = neighbors[rind]->id;

        cout << "N = " << N << endl;
        cout << "new goal_ind = " << goal_ind << endl;
        cout << "p->x = " << p->x << endl;
        cout << "p->y = " << p->y << endl;
		
	}

}*/



//------------- Dijkstra related code ----------------

/*void DijkstrasTest()
{
	Node* n1 = new Node('1', 15, 13);
	Node* n2 = new Node('2', 15, 7);
	Node* n3 = new Node('3', 33, 7);
	Node* n4 = new Node('4', 32.8, 16);
	Node* n5 = new Node('5', 6, 13);
	//Node* f = new Node('6', 0, 0);
	//Node* g = new Node('7', 0, 0);

	Edge* e1 = new Edge(n1, n2, 1); //NEED to reset the distance of the edge
	Edge* e2 = new Edge(n2, n3, 2);
	Edge* e3 = new Edge(n3, n4, 2);
	Edge* e4 = new Edge(n4, n5, 1);
	//Edge* e5 = new Edge(b, f, 3);
	//Edge* e6 = new Edge(c, e, 3);
	//Edge* e7 = new Edge(e, f, 2);
	//Edge* e8 = new Edge(d, g, 1);
	//Edge* e9 = new Edge(g, f, 1);

	n1->distanceFromStart = 0; // set start node
	Dijkstras();
	PrintLoadShortestRouteTo(n4);

	// TODO: Node / Edge memory cleanup not included
}*/

void Dijkstras(map_loader::GraphMap gmap, position_tracker::Position init_pos, position_tracker::Position dst_pos, path_navigator::Waypoints *w)
{
	//vector<map_loader::Node> nodes = gmap.nodes;
	//vector<map_loader::Edge> edges = gmap.edges;

	if(nodes.size() == 0) return;	

	erased.clear();
	previous.clear();
	waypoints.clear();
	erased_num = 0;
	for(int i = 0; i < nodes.size(); i++)
	{
		nodes.at(i).distanceFromStart = 1000000;
		erased.push_back(0);
		//hold the previous node of all the nodes in the shortest path
		previous.push_back(-1);
	}


	
	//find node (init_node) in the graph map that is closest to the init_pos
	vector<map_loader::Node>::iterator itl;
	map_loader::Node init_node, dst_node;
	double min_dist = 10000;
	int min_ind = -1;
        for(itl = nodes.begin(); itl != nodes.end(); ++itl)
	{
		double dist = sqrt(pow(init_pos.x - (*itl).p.x, 2) + pow(init_pos.y - (*itl).p.y, 2));			
		if(dist < min_dist)
		{
			min_dist = dist;
			min_ind = itl - nodes.begin();
		} 
		init_node = nodes[min_ind];		
	}
	std::cout << "Init_pos: It's closest node is (" << init_node.p.x << ", " << init_node.p.y << ")" << std::endl;
	std::cout << "min_ind = " << min_ind << std::endl;
	nodes.at(min_ind).distanceFromStart = 0; // set start node
	

	//find node (dst_node) in the graph map that is closest to the dst_pos
	min_dist = 10000.0;
	min_ind = -1;
	for(itl = nodes.begin(); itl != nodes.end(); ++itl)
	{
		double dist = sqrt(pow(dst_pos.x - (*itl).p.x, 2) + pow(dst_pos.y - (*itl).p.y, 2));			
			if(dist < min_dist)
			{
				min_dist = dist;
				min_ind = itl - nodes.begin();
			} 
		
		dst_node = nodes[min_ind];
	}
	std::cout << "Dst_pos: It's closest node is (" << dst_node.p.x << ", " << dst_node.p.y << ")" << std::endl;
	int dst_min_ind = min_ind;

	//---------------------------------------------


	//while not all the nodes have been examined
	while (erased_num <= nodes.size())
	{

		cout << "sum of erased = " << accumulate(erased.begin(), erased.end(), 0) << endl;

		int smallest_id = ExtractSmallest(nodes);
		cout << "smallest_id = " << smallest_id << endl;
		if(smallest_id == -1) 
		{
			ROS_INFO("No map has been loaded");
			return;
		}

		vector<int> adjacentNodes = AdjacentRemainingNodes(nodes[smallest_id]);
		cout << "number of adjacent nodes = " << adjacentNodes.size() << endl;

		const int size = adjacentNodes.size();
		for (int i=0; i<size; ++i)
		{
			Node adjacent = nodes.at(adjacentNodes.at(i));
			int distance = Distance(nodes.at(smallest_id), adjacent) + nodes.at(smallest_id).distanceFromStart;
			cout << "distance " << distance << " with node " << adjacentNodes.at(i) << endl;
			if (distance < adjacent.distanceFromStart)
			{
				nodes.at(adjacentNodes.at(i)).distanceFromStart = distance;
				previous[adjacent.id] = smallest_id;
			}
		}
	}

	//---------------------------------------------
	PrintLoadShortestRouteTo(nodes[dst_min_ind]); //stores the path in the waypoints vector => 	
 	Node *tmp_dst_node = new Node();
	tmp_dst_node->p = dst_pos;
	waypoints.push_back(*tmp_dst_node); //optional: add also the destination position (assuming it doesn't exist already as a node in the map)
	w->waypoints = waypoints;
}

// Find the node with the smallest distance (that is not examined yet) and return it.
/*Node* ExtractSmallest(vector<Node>& nodes)
{
	int size, smallestPosition;

	size = nodes.size();
	if(size == 0) 
	{
		return NULL;
	}	

	smallestPosition = 0;
	Node smallest = nodes.at(0);
	for (int i=1; i<size; ++i)
	{
		Node current = nodes.at(i);
		if (current.distanceFromStart <
			smallest.distanceFromStart && erased[i] == 0)
		{
			smallest = current;
			smallestPosition = i;
		}
	}
	erased[smallestPosition] = 1;
	
	return &smallest;
}*/

int ExtractSmallest(vector<Node>& nodes)
{
	int size, smallest_id;

	size = nodes.size();
	if(size == 0) 
	{
		return -1;
	}	

	//find first non-erased id
	for(int i=0; i<size; ++i)
	{
		if(erased[i])
		    continue;

		smallest_id = i;
		break;
	}

	for (int i=0; i<size; ++i)
	{
		if (nodes.at(i).distanceFromStart <
			nodes.at(smallest_id).distanceFromStart && erased[i] == 0)
		{
			smallest_id = i;
		}
	}
	erased[smallest_id] = 1;
	erased_num++;
	
	return smallest_id;
}

// Return all nodes adjacent to 'node' which are still
// in the 'nodes' collection.
vector<int> AdjacentRemainingNodes(Node node)
{
	vector<int> adjacentNodes = neighbors.at(node.id);
	cout << "number of neighbors of node.id " << node.id << " = " << adjacentNodes.size() << endl;

	for(int i=adjacentNodes.size()-1; i>=0; --i)
	{
		if(erased[adjacentNodes.at(i)] == 1)
		{
			adjacentNodes.erase(adjacentNodes.begin() + i);
		}
	}
	return adjacentNodes;
}


void calculateNeighbors()
{
	for(int nd = 0; nd < nodes.size(); ++nd)
	{
		Node node = nodes.at(nd);
		vector<int> tmp_neighbors;
		const int size = edges.size();
		for(int i=0; i<size; ++i)
		{
			Edge edge = edges.at(i);
			int adjacent_id = -1;
			//Node adjacent = NULL;
			if (edge.node1_id == node.id)
			{
				adjacent_id = edge.node2_id;
			}
			else if (edge.node2_id == node.id)
			{
				adjacent_id = edge.node1_id;
			}

			if (adjacent_id != -1)
			{
				tmp_neighbors.push_back(adjacent_id);
			}
		}
		neighbors.push_back(tmp_neighbors);
	}

}


// Return distance between two connected nodes
int Distance(Node node1, Node node2)
{
	const int size = edges.size();
	for(int i=0; i<size; ++i)
	{
		Edge edge = edges.at(i);
		if (Connects(edge, node1, node2))
		{
			return edge.distance;
		}
	}
	return -1; // should never happen
}

// Does the 'nodes' vector contain 'node'
bool Contains(vector<Node>& nodes, Node node)
{
	return (!erased[node.id]);
}

//////////////////

void PrintLoadShortestRouteTo(Node destination)
{
	int prev_id = destination.id;

	cout << "Distance from start: " 
		<< destination.distanceFromStart << endl;
	cout << "destination.id = " << destination.id << endl;
	while (prev_id != -1)
	{
		cout << prev_id << " ";
        waypoints.insert(waypoints.begin(), nodes.at(prev_id));
		prev_id = previous[prev_id];
	}
	cout << endl;
}

/*vector<Edge> AdjacentEdges(vector<Edge>& edges, Node node)
{
	vector<Edge> adjacentEdges();

	const int size = edges.size();
	for(int i=0; i<size; ++i)
	{
		Edge edge = edges.at(i);
		if (edge.node1_id == node.id)
		{
			cout << "adjacent: " << edge.node2_id << endl;
			adjacentEdges.push_back(edge);
		}
		else if (edge.node2_id == node.id)
		{
			cout << "adjacent: " << edge.node1_id << endl;
			adjacentEdges.push_back(edge);
		}
	}
	return adjacentEdges;
}*/

/*void RemoveEdge(vector<Edge>& edges, Edge edge)
{
	vector<Edge>::iterator it;
	for (it=edges.begin(); it<edges.end(); ++it)
	{
		if (it == edge)
		{
			edges.erase(it);
			return;
		}
	}
}*/

bool Connects(Edge edge, Node node1, Node node2)
{
	return (
		(node1.id == edge.node1_id &&
		node2.id == edge.node2_id) ||
		(node1.id == edge.node2_id && 
		node2.id == edge.node1_id));
}


