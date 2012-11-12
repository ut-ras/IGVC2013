#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>


class JoystickControl
{
public:
  JoystickControl();

private:
  void joyCallback(const joy::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;
  ros::NodeHandle ph_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};


JoystickControl::JoystickControl():
  ph_("~"),
  linear_(1),
  angular_(2)
{

  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &JoystickControl::joyCallback, this);
}

void JoystickControl::joyCallback(const joy::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_control");
  JoystickControl joystick_control;

  ros::spin();
}

