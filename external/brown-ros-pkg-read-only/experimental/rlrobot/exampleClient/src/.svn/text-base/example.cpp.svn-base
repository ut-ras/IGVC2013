#include <ros/ros.h>
#include <discreteMove/Act.h> //this is imported via the package dependency

//actions
#define LEFT 1
#define FORWARD 2
#define RIGHT 3

bool call(ros::ServiceClient client, int action) {
	discreteMove::Act act;
	act.request.action = action;
	if (client.call(act)) {
		//succeeded
	} else {
		//failed
	}
	return act.response.done;
}

bool left(ros::ServiceClient client) {
	return call(client, LEFT);
}

bool right(ros::ServiceClient client) {
	return call(client, RIGHT);
}

bool forward(ros::ServiceClient client) {
	return call(client, FORWARD);
}

int main(int argc, char **argv) {
	//setup code
	ros::init(argc, argv, "exampleClient");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<discreteMove::Act>("act");

	//calls 
	left(client);
	left(client);
	forward(client);
	right(client);

	return 0;
}
