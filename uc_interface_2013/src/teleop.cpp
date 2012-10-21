/*
 * teleop.cpp
 *
 *  Created on: Apr 27, 2011
 *      Author: ras
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class EnterpRASTeleop
{
public:
	EnterpRASTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
  ros::Time lastTime;
};


EnterpRASTeleop::EnterpRASTeleop():
  linear_(1),
  angular_(2),
  l_scale_(127),
  a_scale_(-127)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("vel_cmd", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &EnterpRASTeleop::joyCallback, this);


}

void EnterpRASTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ROS_DEBUG_STREAM(*joy);
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  
  if(lastTime + ros::Duration(1/20) < ros::Time::now())
  {
  	vel_pub_.publish(twist);
  	lastTime = ros::Time::now();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  EnterpRASTeleop enterpras_teleop;

  ros::spin();
}
