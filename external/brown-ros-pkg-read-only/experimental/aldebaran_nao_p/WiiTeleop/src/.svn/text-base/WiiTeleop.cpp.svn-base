#include <iostream>
#include <stdlib.h>
#include <math.h>

#include <cwiid.h>

#include <ros/ros.h>
#include <NAOcontrol/Head.h>
#include <NAOcontrol/Walk.h>
#include <NAOcontrol/Arm.h>

struct acc_cal wm_cal;
struct acc_cal nk_cal;

using namespace std;

struct msgStruct {
	bool forward;
	bool left;
	bool right;
	bool stand;
	int x;
	int lastx;
	int y;
	int hand;
	int armx;
	int army;
	int armz;
	int wrist;
};

struct msgStruct msg;

cwiid_mesg_callback_t procMessage;

void procMessage(   cwiid_wiimote_t *wiimote,
		    int mesg_count,
		    union cwiid_mesg mesg[],
		    struct timespec *timestamp) {
	float pitch,roll,abspitch;
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
		case CWIID_MESG_ACC:
			//pitch = (mesg[i].acc_mesg.acc[1] - wm_cal.zero[1]) / float(wm_cal.one[1]- wm_cal.zero[1]);
			pitch = float(mesg[i].acc_mesg.acc[1] - 120) / float(25);
			msg.y = 128; //+2*pitch*256;
			//roll = (mesg[i].acc_mesg.acc[0] - wm_cal.zero[0])*fabs(pitch)/float(-(wm_cal.one[0]-wm_cal.zero[0]));
			roll = (mesg[i].acc_mesg.acc[0] - 128)*fabs(pitch)/float(-10);
			if(msg.y > 256) msg.y = 256;
			if(msg.y < 0) msg.y = 0;
			
			msg.x = 128; //+ 256*roll;
			if(msg.x > 256) msg.x = 256;
			if(msg.x < 0) msg.x = 0;
			//cout << "Set head to " << msg.x << " " << msg.y << endl;
			break;

		case CWIID_MESG_NUNCHUK:
			msg.armx = mesg[i].nunchuk_mesg.stick[CWIID_Y];
			msg.army = mesg[i].nunchuk_mesg.stick[CWIID_X];
			pitch = float(mesg[i].nunchuk_mesg.acc[1] - nk_cal.zero[1]) / float(nk_cal.one[1]-nk_cal.zero[1]);
			abspitch = fabs(pitch);
			if (abspitch > .9)
				abspitch = .9;
			roll = (mesg[i].nunchuk_mesg.acc[0] - nk_cal.zero[0])/float(nk_cal.one[0]-nk_cal.zero[0])/(1.1-abspitch);
			msg.wrist = 128+roll*128;
			msg.armz = 128+pitch*128;
			if(msg.armz > 256) msg.armz = 256;
			if(msg.armz > 110 && msg.armz < 156) msg.armz = 128;
			if(msg.armz < 0) msg.armz = 0;
			if(msg.wrist > 256) msg.wrist = 256;
			if(msg.wrist > 110 && msg.wrist < 156) msg.wrist = 128;
			if(msg.wrist < 0) msg.wrist = 0;
			if (mesg[i].nunchuk_mesg.buttons & CWIID_NUNCHUK_BTN_C) {
				msg.hand = 0;
			} else if (mesg[i].nunchuk_mesg.buttons & CWIID_NUNCHUK_BTN_Z) {
				msg.hand = 1;
			} else {
				msg.hand = 2;
			}




			break;
	}
    }

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "wiiteleop");
	ros::NodeHandle n;
	ros::Publisher head_pub = n.advertise<NAOcontrol::Head>("head", 30);
	ros::Publisher motion_pub = n.advertise<NAOcontrol::Walk>("motion", 1);
	ros::Publisher arm_pub = n.advertise<NAOcontrol::Arm>("RArm", 1);

	cout << "Press 1+2 (at the same time) on the Wiimote now!" << endl;
	cwiid_wiimote_t *wiimote;
	bdaddr_t bdaddr = *BDADDR_ANY;
	wiimote = cwiid_open(&bdaddr,0);
	wm_cal.zero[0] = 23;
    	if(cwiid_get_acc_cal(wiimote, CWIID_EXT_NONE, &wm_cal)){
		cout << "Could not get calibration info" << endl;
	}
	else{
		cout << "Cal Specs "<< int(wm_cal.zero[0]) << " " << int(wm_cal.zero[1]) << " " <<int(wm_cal.zero[2]) << endl;
	}
	if(cwiid_get_acc_cal(wiimote, CWIID_EXT_NUNCHUK, &nk_cal)){
		cout << "Could not get calibration info" << endl;
	}
	else{
		cout << "Cal Specs "<< int(nk_cal.zero[0]) << " " << int(nk_cal.zero[1]) << " " <<int(nk_cal.zero[2]) << endl;
	}
	cwiid_set_mesg_callback(wiimote,procMessage);
	cwiid_set_rpt_mode(wiimote,CWIID_RPT_ACC|CWIID_RPT_BTN|CWIID_RPT_EXT);
	std::cout << "Thank you." << std::endl;

	cwiid_enable(wiimote, CWIID_FLAG_MESG_IFC);


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

		NAOcontrol::Arm armPos;
		armPos.hand = msg.hand;
		armPos.armPose[0] = msg.armx;
		armPos.armPose[1] = msg.army;
		armPos.armPose[2] = msg.armz;
		armPos.armPose[3] = msg.wrist;
		armPos.mask = 15;
		armPos.duration = .1;
		arm_pub.publish(armPos);
		loop_rate.sleep();
	}


    	return 0;

}
