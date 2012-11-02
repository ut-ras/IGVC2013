Author: Gal Peleg

This package is geared at 'getting the current joint angles for the PR2,' and is based off a tutorial by the same name 
(http://www.ros.org/wiki/pr2/Tutorials/Getting%20the%20current%20joint%20angles)

Operation Instructions:

1. Run the server:	python src/joint_states_listener/joint_states_listener.py

2. To actually get information about the joints, run the test client:	python scripts/joint_states_listener_test.py


Two things of note:

1. The code in the test client (i.e., scripts/joint_states_listener_test.py) is just meant to serve as an example. Look into it to see just how the develpor (we) can tweek it to look, say, at different joints, other paramters, etc.

2. This package comprises really just of a 'handy' impelementation of a listener/server. There ARE other ways of going about reteiving this kind of information. In particular,

"The current positions and angles for the robot are by default being broadcast on a message topic called joint_states, of type sensor_msgs/JointState. This message contains arrays of joint names (string[] name), along with the corresponding positions (float64[] position), velocities (float64[] velocity), and efforts (float64[] effort). If you type

rostopic echo joint_states

at the command line after a real or simulated robot has been launched, you can see these messages as they are published."
