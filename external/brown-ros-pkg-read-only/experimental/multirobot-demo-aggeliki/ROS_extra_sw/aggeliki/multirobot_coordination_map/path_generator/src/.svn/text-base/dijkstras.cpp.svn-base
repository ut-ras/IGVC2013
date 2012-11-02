#include "dijkstras.h"

/*void Dijkstras(map_loader::GraphMap gmap, position_tracker::Position init_pos, position_tracker::Position dst_pos, path_navigator::Waypoints *w)
{
	//vector<map_loader::Node> nodes = gmap.nodes;
	//vector<map_loader::Edge> edges = gmap.edges;
	
	//hold the previous node of all the nodes in the shortest path
	vector<map_loader::Node> *previous_tmp = new vector<map_loader::Node>(nodes.size()); 
	previous = *previous_tmp;	
	
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
		
		std::cout << "Init_pos: It's closest node is (" << init_node.p.x << ", " << init_node.p.y << ")" << std::endl;
	}

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
		
		std::cout << "Dst_pos: It's closest node is (" << dst_node.p.x << ", " << dst_node.p.y << ")" << std::endl;
	}
	

	//---------------------------------------------

	//while not all the nodes have been examined
	while (accumulate(erased.begin(), erased.end(), 0) <= nodes.size())
	{
		Node smallest = ExtractSmallest(nodes);
		vector<int> adjacentNodes = AdjacentRemainingNodes(smallest);

		const int size = adjacentNodes.size();
		for (int i=0; i<size; ++i)
		{
			Node adjacent = nodes.at(adjacentNodes.at(i));
			int distance = Distance(smallest, adjacent) +
				smallest.distanceFromStart;
			
			if (distance < adjacent.distanceFromStart)
			{
				adjacent.distanceFromStart = distance;
				previous[adjacent.id] = smallest;
			}
		}
	}

	//---------------------------------------------
	init_node.distanceFromStart = 0; // set start node
	PrintLoadShortestRouteTo(dst_node); //stores the path in the waypoints vector => 	

	w->waypoints = waypoints;
}

// Find the node with the smallest distance (that is not examined yet) and return it.
Node ExtractSmallest(vector<Node>& nodes)
{
	int size = (int)nodes.size();
	if(size == 0) 
	{
		return NULL;
	}	

	int smallestPosition = 0;
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
	
	return smallest;
}

// Return all nodes adjacent to 'node' which are still
// in the 'nodes' collection.
vector<int> AdjacentRemainingNodes(Node node)
{
	vector<int> adjacentNodes = neighbors.at(node.id);
	for(int i=adjacentNodes.size(); i>=0; --i)
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
		vector<int> tmp_neighbors();
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

///////////////////

void PrintLoadShortestRouteTo(Node destination)
{
	Node prev = destination;

	cout << "Distance from start: " 
		<< destination.distanceFromStart << endl;
	cout << "destination.id = " << destination.id << endl;
	while (prev != NULL)
	{
		cout << prev.id << " ";
        waypoints.insert(waypoints.begin(), prev);
		prev = previous[prev.id];
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

/*bool Connects(Edge edge, Node node1, Node node2)
{
	return (
		(node1.id == edge.node1_id &&
		node2.id == edge.node2_id) ||
		(node1.id == edge.node2_id && 
		node2.id == edge.node1_id));
}*/

