------------------------------------------------------------
------------------------------------------------------------
Overview:
------------------------------------------------------------
	This collection of ROS packages was designed to provide rudimentary
control of the Aldebaran NAO and to provide access to its forehead camera. The
motivation is to allow for basic navigation and vision experiments with ROS. We
hope to add more capabilities in response to the community's needs.

The following packages are included:

NAOcontrol
naoTalk
NaoImageTransport
WiiTeleop

	NAOcontrol provides the basic messages needed for moving the robot.
naoTalk provides command line control of the NAO's text-to-speech capabilities.
NaoImageTransport acts as a bridge between our NAO driver and ROS exporting a
standard ROS image transport suitable for use with ROS utilities such as
image_view and cv_blob (due to bandwidth and processing limits, our NAO driver
does not itself export ROS native image transports). WiiTeleop is meant as an
example NAO controlling ROS node. It allows a user to control the NAO's head
position and movements via WiiMote.

	In addition to these packages we've included a number of items that need
to be installed on your NAO. These are located in the naoNative folder.

------------------------------------------------------------
------------------------------------------------------------
Building The ROS Packages:
------------------------------------------------------------

	For the most part the packages should build with rosmake as expected.
The total dependencies of the packages are as follows:

cv_bridge
cwiid
diagnostic_msgs
diagnostic_updater
genmsg_cpp
geometry_msgs
image_transport
joy
message_filters
opencv2
paramiko
pluginlib
pycrypto
rosbagmigration
rosconsole
roscpp
roslang
roslaunch
roslib
rosout
rospack
rospy
rosrecord
rostest
sensor_msgs
std_msgs
std_srvs
tinyxml
topic_tools
wiimote
xmlrpcpp

	All of these packages are available through svn from Willow Garage
itself at http://ros.org .

------------------------------------------------------------
------------------------------------------------------------
Installing the NAO driver:
------------------------------------------------------------

	In addition to the usual ROS packages, you must also install and run our
NAO driver on your NAO. It is located in naoNative. To install it:

1) scp libbrowneyes.so to /opt/naoqi/modules/lib on your NAO.

2) Add browneyes to your NAO's autoload.ini (also located in
/opt/naoqi/modules/lib ). We recommend adding it right before the [remote]
directive. We have provided an example autoload.ini . This file is for example
purposes only and should *not* be copied verbatim to your NAO.

3) untar naoros.tar to an easily SSH accessible portion of your NAO. We
recommend /home/root .

4) Inside the resulting naoros/run directory should be a file named envConfig,
this file should be edited to reflect your configuration (e.g. naoros's absolute
path and the hostname where you intend to run roscore). 

You can now run the NAO driver:

1) SSH into your NAO.
2) go to your naoros/run directory.
3) source envConfig
4) ./eyes &
5) python control.py 

If you wish you can run this last command as:

python control.py &

allowing you to log out of your NAO.

------------------------------------------------------------
------------------------------------------------------------
Tips:
------------------------------------------------------------

1) Wiimote controls are as follows:

Numchuck (required) controls the head
left and right on the d-pad controls left and right
trigger (B) on the Wiimote controls forward

2) control.py looks for a file named walkConfig.py inside naoros/run that
defines a walkConfig for the NAO (see our example walkConfig.py and the
Aldebaran documentation). We *highly* recommend you provide such a customized
file. In our experience there is a great variety from NAO to NAO and from
surface to surface in their ability to walk correctly with the default walk.

3) NaoImageTransport will make use of any image_transport plugins you have
available.

4) The Walk msg can be thought of as bitmask consisting of forward (1st bit),
left (2nd bit), and right (3rd bit). In other words:
1 - forward only
2 - left only
3 - forward AND left
4 - right only
5 - forward AND right

5) Don't forget that your NAO and the machine running roscore must be able to
identify each other *by name*. This may require adding entries to the /etc/hosts
files.

