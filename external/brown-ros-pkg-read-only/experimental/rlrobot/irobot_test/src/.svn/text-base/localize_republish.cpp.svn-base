#include <ros/ros.h>
#include <iostream>
#include <irobot_test/LocalizableObject.h>
#include <irobot_test/Locations.h>
#include "../include/global.h"
#include "../include/udpsocket.h"
#include "../include/datagram.h"
#include "../include/objectposepacket.h"
#include "../include/localizableobject.h"
#include "../include/objecttype.h"

using namespace std;
using namespace rlab;

int main(int argc, char* argv[])
{
  // Create UDP socket that listens on port 8856 and is non-blocking
  UDPSocket udpSocket(8856, false);
   if (!udpSocket.prepareUDPServerSocket()) {
    cerr << "Failed to create UDP socket on port " << udpSocket.getPort() << "." << endl;
    return 1;
  }
  cout << "Listening on port " << udpSocket.getPort() << "..." << endl;
  
    ros::init(argc, argv, "Location Manager");
  ros::NodeHandle n;
  ros::Publisher position_pub = n.advertise<irobot_test::Locations>("topDownLocations", 100);
  ros::Rate loop_rate(5);
  
  while (true) {
    while(ros::ok()){
      // Read packets as long as there are some available
     
      irobot_test::Locations locations;

      while (udpSocket.dataAvailable()) {
	Datagram* datagram = udpSocket.receiveDatagram();
	ObjectPosePacket* packet = ObjectPosePacket::parse(datagram->getMessage());
       
	
	if (!packet) {
	  cerr << "Received invalid UDP packet." << endl;
	  delete datagram;
	  continue;
	}	


	cout << "Content of packet from " << datagram->getPeerInfo() << ":" << endl;
	for (unsigned int i = 0; i < packet->getLocalizableObjectCount(); ++i) {
	  const LocalizableObject& object = packet->getLocalizableObject(i);
	  irobot_test::LocalizableObject ros_object;

	  cout << "Object type: ";
	  if (object.getType() == LocalizableObject::OBJECT_TYPE_SMURV) {
	    ros_object.objecttype=OBJ_TYPE_SMURV;
	    cout << "SmURV "<< OBJ_TYPE_SMURV << " ";
	    
	  }
	  else if (object.getType() == LocalizableObject::OBJECT_TYPE_BALL) {
	    ros_object.objecttype=OBJ_TYPE_BALL;
	    cout << "Ball "<< OBJ_TYPE_BALL << " ";
	  }
	  else {
	    ros_object.objecttype=OBJ_TYPE_OBSTACLE;
	    cout << "Obstacle" << "SmURV "<< OBJ_TYPE_OBSTACLE << " ";
	  }
	  cout << ros_object.objecttype;
	  cout << ", ID: " << object.getId();
	  cout << ", X: " << object.getPose().getX();
	  cout << ", Y: " << object.getPose().getY();
	  cout << ", Yaw: " << object.getPose().getYaw();
	  cout << ", age of pose in ms: " << object.getPoseAge() << endl;
	  //	  ros_object.objecttype=object.getType();
	  ros_object.objectid=object.getId();
	  ros_object.posx=object.getPose().getX();
	  ros_object.posy=object.getPose().getY();
	  ros_object.yaw=object.getPose().getYaw();
	  locations.objectlist.push_back(ros_object);
		  
	}
	cout << "------------------------------------------------------" << endl;
	
	delete packet;
	delete datagram;
      }
      
      //send messages to ros
   
      position_pub.publish(locations);
      
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
    
  return 0;
}

