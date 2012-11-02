#include "map_loader.h"
#include "map_loader/Tokenizer.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "map_loader");
     ros::NodeHandle n1, n2, n3, n4;
     lines_pub = n1.advertise<map_loader::LineMap>("/line_map", 1); //SET QUEUE SIZE = 1!!! (keep only last msg)
     graph_pub = n1.advertise<map_loader::GraphMap>("/graph_map", 1); //SET QUEUE SIZE = 1!!! (keep only last msg)

	 loadMap();
	 while(1)
	 {
		lines_pub.publish(*lmap);
		//cout << "lmap->lines.size() = " << lmap->lines.size() << endl;
		graph_pub.publish(*gmap);
		ros::spinOnce();
	 }
	 //ros::spinOnce();
	 //sleep???
}

void loadMap()
{
	lmap = new map_loader::LineMap();
	gmap = new map_loader::GraphMap();

	vector<map_loader::Node> nodes;
	vector<map_loader::Edge> edges;
	vector<map_loader::Line> map4;

    //note: ordering of points when adding edges: from left to right, from bottom to top
	map4.clear();
	nodes.clear();
	edges.clear();

	//----------------- load lines map -----------------------
	//NEED RELATIVE PATH !!!!!!!!!
	ifstream ifs("/opt/ros/boxturtle/ros/aggeliki/map_loader/bin/lines_map.txt");
	string line;
	if(!ifs)
	{
		cerr << "Error: file could not be opened" << endl;
		exit(1);
	}
	
	while(!ifs.eof())
	{
		getline(ifs, line);
		cout << "[" << line << "]" << endl;
	
		if(line == "")
			break;

		//tokenize line
		string delim = "\t";
        Tokenizer str(line, delim);

		//while((token = str.next())! = "");
		double Ax = strtod(str.next().c_str(), NULL);
		double Ay = strtod(str.next().c_str(), NULL);
		double Bx = strtod(str.next().c_str(), NULL);
		double By = strtod(str.next().c_str(), NULL);
		double theta = strtod(str.next().c_str(), NULL);	//ignore the rest of the line
		
		cout << "Ax = " << Ax << ", Bx = " << Bx << ", Ay = " << Ay << ", By = " << By << ", theta = " << theta << endl;

		map_loader::Line *l = new map_loader::Line();
		l->Ax = Ax;
		l->Ay = Ay;
		l->Bx = Bx;
		l->By = By;
		l->theta = theta;
		
 		map4.push_back(*l);
 	}
	ifs.close();

	cout << "map4.size() = " << map4.size() << endl;
	lmap->lines = map4;
	cout << "lmap->lines.size() = " << lmap->lines.size() << endl;


	//----------------- load graph map -----------------------
	//NEED RELATIVE PATH !!!!!!!!!
	ifstream ifs2("/opt/ros/boxturtle/ros/aggeliki/map_loader/bin/nodes_map.txt");
	if(!ifs2)
	{
		cerr << "Error: file could not be opened" << endl;
		exit(1);
	}
	
	while(!ifs2.eof())
	{
		getline(ifs2, line);
		cout << "[" << line << "]" << endl;
	
		if(line == "")
			break;

		//tokenize line
		string delim = "\t";
        Tokenizer str(line, delim);

		//while((token = str.next())! = "");
		int id = atoi(str.next().c_str());
		double x = strtod(str.next().c_str(), NULL);
		double y = strtod(str.next().c_str(), NULL);
		
		cout << "id = " << id << ", x = " << x << ", y = " << y << endl;

		map_loader::Node *n = new map_loader::Node();
		n->id = id;
		n->distanceFromStart = INT_MAX;
		position_tracker::Position *p = new position_tracker::Position();
		p->x = x;
		p->y = y;
		n->p = *p;
		
		nodes.push_back(*n);
 	}
	ifs2.close();

	cout << "nodes.size() = " << nodes.size() << endl;
	gmap->nodes = nodes;

	//read edges as well
	ifstream ifs3("/opt/ros/boxturtle/ros/aggeliki/map_loader/bin/edges_map.txt");
	if(!ifs3)
	{
		cerr << "Error: file could not be opened" << endl;
		exit(1);
	}
	
	while(!ifs3.eof())
	{
		getline(ifs3, line);
		cout << "[" << line << "]" << endl;
	
		if(line == "")
			break;

		//tokenize line
		string delim = "\t";
        Tokenizer str(line, delim);

		//while((token = str.next())! = "");
		int id1 = atoi(str.next().c_str());
		int id2 = atoi(str.next().c_str());
		double dist = strtod(str.next().c_str(), NULL);
		
		cout << "id1 = " << id1 << ", id2 = " << id2 << ", dist = " << dist << endl;

		map_loader::Edge *e = new map_loader::Edge();
		e->node1_id = id1;
		e->node2_id = id2;
		e->distance = dist;
		
 		edges.push_back(*e);
 	}
	ifs3.close();

	cout << "edges.size() = " << edges.size() << endl;
	gmap->edges = edges;
	
/*   //INPUT THE MAP IN THE FORM OF NODES
    //SOS: id's of nodes should be in the order the nodes are created, they should start from 0
	//--------- corner nodes ----------
	Node* n1 = new Node(0, 7, 13); nodes.push_back(*n1);
	Node* n2 = new Node(1, 15, 13); nodes.push_back(*n2);
	Node* n3 = new Node(2, 15, 7); nodes.push_back(*n3);
	Node* n4 = new Node(3, 33, 7); nodes.push_back(*n4);
	Node* n5 = new Node(4, 32.8, 16); nodes.push_back(*n5);

	//-- intermediate nodes, corridor 1 --
	Node* n6 = new Node(5, 8.5, 13); nodes.push_back(*n6);
	Node* n7 = new Node(6, 10, 13); nodes.push_back(*n7);
	Node* n8 = new Node(7, 11.5, 13); nodes.push_back(*n8);
	Node* n9 = new Node(8, 13, 13); nodes.push_back(*n9);

	//-- intermediate nodes, corridor 2 --
	Node* n10 = new Node(9, 15, 11.5); nodes.push_back(*n10);
	Node* n11 = new Node(10, 15, 10); nodes.push_back(*n11);
	Node* n12 = new Node(11, 15, 8.5); nodes.push_back(*n12);

	//-- intermediate nodes, corridor 3 (DON'T CARE FOR NOW) --
	Node* n13 = new Node(12, 16.5, 7); nodes.push_back(*n13);
	Node* n14 = new Node(13, 18, 7); nodes.push_back(*n14);
	Node* n15 = new Node(14, 19.5, 7); nodes.push_back(*n15);
	Node* n16 = new Node(15, 21, 7); nodes.push_back(*n16);
	Node* n17 = new Node(16, 22.5, 7); nodes.push_back(*n17);
	Node* n18 = new Node(17, 24, 7); nodes.push_back(*n18);*/

/*	//Edge* e1 = new Edge(n1, n2, 1); //NEED to reset the distance of the edge
	Edge* e2 = new Edge(n2, n3, 2);
	Edge* e3 = new Edge(n3, n4, 2);
	Edge* e4 = new Edge(n4, n5, 1);
	Edge* e5 = new Edge(n1, n6, 1);
	Edge* e6 = new Edge(n6, n7, 1);
	//Edge* e7 = new Edge(n7, n8, 1);
	Edge* e8 = new Edge(n7, n2, 1);*/

/*	Edge* e1 = new Edge(n1, n6, 1); //NEED to reset the distance of the edge
	Edge* e2 = new Edge(n6, n7, 1);
	Edge* e3 = new Edge(n7, n8, 1);
	Edge* e4 = new Edge(n8, n9, 1);
	Edge* e5 = new Edge(n9, n2, 1);
	Edge* e6 = new Edge(n2, n10, 1);
	Edge* e7 = new Edge(n10, n11, 1);
	Edge* e8 = new Edge(n11, n12, 1);
	Edge* e9 = new Edge(n12, n3, 1);
	Edge* e10 = new Edge(n3, n13, 1);
	Edge* e11 = new Edge(n13, n14, 1);
	Edge* e12 = new Edge(n14, n15, 1);
	Edge* e13 = new Edge(n15, n16, 1);
	Edge* e14 = new Edge(n16, n17, 1);
	Edge* e15 = new Edge(n17, n18, 1);
	Edge* e16 = new Edge(n18, n4, 1);
	Edge* e17 = new Edge(n4, n5, 1);
*/

}

void error(char *msg)
{
    perror(msg);
    exit(1);
}

