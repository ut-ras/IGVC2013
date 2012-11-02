#include <iostream>
#include <stdlib.h>

#include <cwiid.h>

#include <ros/ros.h>
#include "naoExpmnt/Head.h"
#include "naoExpmnt/Motion.h"

struct acc_cal wm_cal;

using namespace std;

struct msgStruct {
	bool forward;
	bool left;
	bool right;
	bool stand;
	int x;
	int lastx;
	int y;
};

struct msgStruct msg;

cwiid_mesg_callback_t procMessage;

void procMessage(   cwiid_wiimote_t *wiimote,
		    int mesg_count,
		    union cwiid_mesg mesg[],
		    struct timespec *timestamp) {

    for (int i=0; i < mesg_count; i++) {
	switch(mesg[i].type) {
		case CWIID_MESG_BTN:
			if (mesg[i].btn_mesg.buttons & CWIID_BTN_B) {
				msg.forward = true;
			} else {
				msg.forward = false;
			}
			if (mesg[i].btn_mesg.buttons & CWIID_BTN_LEFT)  {
				msg.left = true;
			} else {
				msg.left = false;
			}
			if (mesg[i].btn_mesg.buttons & CWIID_BTN_RIGHT)  {
				msg.right = true;
			} else {
				msg.right = false;
			}
			if (mesg[i].btn_mesg.buttons & CWIID_BTN_A) {
				msg.stand = true;
			} else {
				msg.stand = false;
			}
			break;
		case CWIID_MESG_NUNCHUK:
			msg.x = mesg[i].nunchuk_mesg.stick[CWIID_X];
			msg.y = mesg[i].nunchuk_mesg.stick[CWIID_Y];





			break;
	}
    }

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "wiihead");
	ros::NodeHandle n;
	ros::Publisher head_pub = n.advertise<naoExpmnt::Head>("head", 1);
	ros::Publisher motion_pub = n.advertise<naoExpmnt::Motion>("motion", 1);

	cout << "Press 1+2 (at the same time) on the Wiimote now!" << endl;
	cwiid_wiimote_t *wiimote;
	bdaddr_t bdaddr = *BDADDR_ANY;
	wiimote = cwiid_open(&bdaddr,0);
	cwiid_set_mesg_callback(wiimote,procMessage);
	cwiid_set_rpt_mode(wiimote,CWIID_RPT_ACC|CWIID_RPT_BTN|CWIID_RPT_EXT);
	std::cout << "Thank you." << std::endl;

	cwiid_enable(wiimote, CWIID_FLAG_MESG_IFC);
    	cwiid_get_acc_cal(wiimote, CWIID_EXT_NONE, &wm_cal);

	ros::Rate loop_rate(30);

	while(ros::ok()) {
		naoExpmnt::Head headPos;
		headPos.x = msg.x;
		headPos.y = msg.y;
		head_pub.publish(headPos);	

		naoExpmnt::Motion motion;
		motion.forward = motion.left = motion.right = motion.stand = 0;
		if (msg.forward) motion.forward = 1;
		if (msg.left) motion.left = 1;
		if (msg.right) motion.right = 1;
		if (msg.stand) motion.stand = 1;
		motion_pub.publish(motion);

		loop_rate.sleep();
	}


    	return 0;

}
