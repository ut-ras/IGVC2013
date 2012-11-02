#include <iostream>
#include <stdlib.h>

#include <cwiid.h>

#include <ros/ros.h>
#include <NAOcontrol/Head.h>
#include <NAOcontrol/Walk.h>

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
	ros::init(argc, argv, "wiiteleop");
	ros::NodeHandle n;
	ros::Publisher head_pub = n.advertise<NAOcontrol::Head>("head", 30);
	ros::Publisher motion_pub = n.advertise<NAOcontrol::Walk>("motion", 1);

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
		NAOcontrol::Head headPos;
		headPos.x = msg.x;
		headPos.y = msg.y;
		head_pub.publish(headPos);	

		NAOcontrol::Walk motion;
		motion.walk = 0;
		if (msg.forward) motion.walk += 1;
		if (msg.left) motion.walk += 2;
		if (msg.right) motion.walk += 4;
		motion_pub.publish(motion);

		loop_rate.sleep();
	}


    	return 0;

}
