/* -------------------------------------------------------------------------- */
// BMA180 Spi Communication Node
//
//    - Reads from Shared Memory buffer
//    - Publishes message with accel data
//
//  Nikhil Deshpande, 7-Jul-2011 | Robert Bosch RTC
/* -------------------------------------------------------------------------- */

// Includes
/* -------------------------------------------------------------------------- */
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <linux/spi/spidev.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <gumstix_memread_bma180/bma180meas.h>
#include <gumstix_memwrite_bma180/writermsg.h>
#include <errno.h>

using namespace std;

#define SHMSZ	30720000
#define xSHMSZ	0x1D4C000  //hex converted
#define NUM_SENSORS	8	
bool readit=false;

void msgCallback(const gumstix_memwrite_bma180::writermsg::ConstPtr & msg){
	if(msg->written==true){
		readit=true;
	} else{
		readit=false;
	}
}

// Main

int main(int argc, char** argv) {
	// Ros
	ros::init(argc, argv, "gumstix_memread_bma180");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<gumstix_memread_bma180::bma180meas> ("bma180meas", 1000);
	ros::Subscriber sub = n.subscribe("/writermsg", 100, msgCallback);

	// Initialize ROS Message
	gumstix_memread_bma180::bma180meas msg;

	// Rate object
	double rate_Hz;
	if(n.getParam("/gumstix_memread_bma180/rate_Hz", rate_Hz)==false){
		rate_Hz = 50;
	}	
	cout << "Publishing rate: " << rate_Hz << " Hz" << endl;	
	// Initialize Shared Memory
	int16_t *shm, *s, shmid;
	key_t key = 42;
	if ((shmid = shmget(key, SHMSZ, 0666)) < 0) {
	    cerr << "shmget did not work - " << errno << endl;
            if(errno == EINVAL)
                printf("Invalid segment size specified\n");
            else if(errno == EEXIST)
                printf("Segment exists, cannot create it\n");
            else if(errno == EIDRM)
                printf("Segment is marked for deletion or was removed\n");
            else if(errno == ENOENT)
                printf("Segment does not exist\n");
            else if(errno == EACCES)
                printf("Permission denied\n");
            else if(errno == ENOMEM)
                printf("Not enough memory to create segment\n");
	}
	if ((shm = (int16_t *)shmat(shmid, NULL, 0)) == (int16_t *) -1) {
    		cerr << "shmat did not work!" << endl;	
	}

	// J counts measurements per Message (index in msg array)
	int16_t i=0, j=0, k=0, val=0;
	long m=0, l=0;
	s = shm;
	s++;
	cout << "Begin memory <reader> : " << s << endl;
	ros::Rate loop1(1);
	while(ros::ok()){
		if(readit==true){
			readit = false;
			break;
		}
		ros::spinOnce();
		loop1.sleep();
	}
	cout << "Publishing to topic /bma180meas... " << endl;
	ros::Rate loop_rate(rate_Hz);
	// Main loop
	while (ros::ok()) {
		// Transfer/receive messages
		int num_msgs = 6*10*NUM_SENSORS;
		for (j=0; j<(num_msgs); j++) {
			i  = (int16_t)*s;
			msg.vals.push_back(i);
			//cout << "value: " << msg.vals.at(j) << endl;
			s++; m++;
			if(s>=(shm+xSHMSZ)){
				s = shm;
				s++;
			}
			if(i==0) l++;
			if(l>=num_msgs){
				k=1;
				break;
			}
		}
		l=0;
		pub.publish(msg);
		msg.vals.clear();
		if(k==1){
			break;
		}
		// Ros rate
		ros::spinOnce();
		loop_rate.sleep();	
	}
	cout << "Done publishing..." << endl;
	cout << "Values published: " << m << endl; 
//	*shm = m+1;
	cout << "End memory <reader>: " << s << endl;
	exit(0);

}
