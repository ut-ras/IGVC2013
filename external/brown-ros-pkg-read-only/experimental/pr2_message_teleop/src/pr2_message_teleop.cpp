#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "pr2_message_teleop/pr2_message_teleop_commander.h"


GeneralCommander* gc;
  
double max_pan_, max_tilt_, min_tilt_;
double tilt_scale_, pan_scale_;

double des_pan_pos_;
double des_tilt_pos_;

bool head_init_;

double min_torso_;
double max_torso_;
double des_torso_pos_;
double torso_step_;

double des_vx_;
double des_vy_;
double des_vw_;

double vx_scale_;
double vy_scale_;
double vw_scale_;

double arm_x_scale_;
double arm_y_scale_;
double arm_z_scale_;

double wrist_velocity_;

void sendHeadCommand(double pan_diff, double tilt_diff) {
  if(!head_init_) {
    double cur_pan_pos = 0.0;
    double cur_tilt_pos = 0.0;
    bool ok = gc->getJointPosition("head_pan_joint", cur_pan_pos);
    if(!ok) return;
    ok = gc->getJointPosition("head_tilt_joint", cur_tilt_pos);
    if(!ok) return;
    des_pan_pos_ = cur_pan_pos;
    des_tilt_pos_ = cur_tilt_pos;
    head_init_ = true;
  }
  des_pan_pos_ += pan_diff;
  des_pan_pos_ = std::min(des_pan_pos_, max_pan_);
  des_pan_pos_ = std::max(des_pan_pos_, -max_pan_);
  des_tilt_pos_ += tilt_diff;
  des_tilt_pos_ = std::min(des_tilt_pos_, max_tilt_);
  des_tilt_pos_ = std::max(des_tilt_pos_, min_tilt_);
  gc->sendHeadCommand(des_pan_pos_, des_tilt_pos_);
}

void handleCommand(const std_msgs::UInt8::ConstPtr& msg) {

  char command = (char)msg->data;

  switch(command) {

    // Base commands
  case 38: // 37-40 are arrow keys
    gc->sendBaseCommand(vx_scale_, 0.0, 0.0);
    break;
  case 40:
    gc->sendBaseCommand(-vx_scale_, 0.0, 0.0);
    break;
  case 37:
    gc->sendBaseCommand(0.0, 0.0, vw_scale_);
    break;
  case 39:
    gc->sendBaseCommand(0.0, 0.0, -vw_scale_);
    break;
  case 'Z':
    gc->sendBaseCommand(0.0, vy_scale_, 0.0);
    break;
  case 'X':
    gc->sendBaseCommand(0.0, -vy_scale_, 0.0);
    break;
    // Head commands
  case 'W':
    sendHeadCommand(0.0, -tilt_scale_);
    break;
  case 'S':
    sendHeadCommand(0.0, tilt_scale_);
    break;
  case 'A':
    sendHeadCommand(pan_scale_, 0.0);
    break;
  case 'D':
    sendHeadCommand(-pan_scale_, 0.0);
    break;
    // Arm controls (right arm only, for now)
  case 'O':
    gc->sendGripperCommand(GeneralCommander::ARMS_RIGHT, false);
    break;
  case 'P':
    gc->sendGripperCommand(GeneralCommander::ARMS_RIGHT, true);
    break;
  case 'I':
    gc->sendArmVelCommands(arm_x_scale_,0.0,0.0,0.0,
			   0.0, 0.0,0.0,0.0,
			   20.0);
    break;
  case 'K':
    gc->sendArmVelCommands(-arm_x_scale_,0.0,0.0,0.0,
			   0.0, 0.0,0.0,0.0,
			   20.0);
    break;
  case 'J':
    gc->sendArmVelCommands(0.0,arm_y_scale_,0.0,0.0,
			   0.0, 0.0,0.0,0.0,
			   20.0);
    break;
  case 'L':
    gc->sendArmVelCommands(0.0,-arm_y_scale_,0.0,0.0,
			   0.0, 0.0,0.0,0.0,
			   20.0);
    break;
  case 'H':
    gc->sendArmVelCommands(0.0,0.0,arm_z_scale_,0.0,
			   0.0, 0.0,0.0,0.0,
			   20.0);
    break;
  case 'N':
    gc->sendArmVelCommands(0.0,0.0,-arm_z_scale_,0.0,
			   0.0, 0.0,0.0,0.0,
			   20.0);
    break;

  }

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "pr2_message_teleop");
  ros::NodeHandle nh;

  gc = new GeneralCommander(true, true, true, true, false);

  max_pan_ = 2.7;
  max_tilt_ = 1.4;
  min_tilt_ = -0.4;
  tilt_scale_ = 0.05;
  pan_scale_ = 0.05;
  vx_scale_ = 0.6;
  vy_scale_ = 0.6;
  vw_scale_ = 0.8;
  torso_step_ = 0.01;
  min_torso_ = 0.0;
  max_torso_ = 1.8;
  arm_x_scale_ = 0.05;
  arm_y_scale_ = 0.05;
  arm_z_scale_ = 0.05;
  wrist_velocity_ = 4.5;
  head_init_ = true;

  ros::Subscriber sub = nh.subscribe("pr2_message_teleop/command", 10, handleCommand);
  ros::spin();

  return(0);
}
