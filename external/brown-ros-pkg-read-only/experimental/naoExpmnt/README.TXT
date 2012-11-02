NaoExpmnt is experimental proof of concept software and is not yet available as
a normal ROS package (e.g. there is not as of yet integration with ROS's OpenCV
package). However, the following steps should get you started.

########################################
You will need the development headers 
and libraries for:
########################################
OpenCV
CWIID

########################################
Getting Ready on the NAO:
########################################
1) scp and untar naoros.tar in a NAO user's home directory

########################################
Compiling the Teleoperating Nodes:
########################################
1) Uncomment and correct the example include_directories and link_directories
statements so that rosmake will be able to find your opencv and cwiid headers
and libraries.
2) Overwrite manifest.xml with manifest.xml.actual.
3) run rosmake

########################################
Teleoperating the NAO
########################################
1) Start roscore 

2) SSH into the NAO and:
3) cd into naoros/scripts
3) Define the environmental variable ROS_MASTER_URI as appropriate (e.g. export
ROS_MASTER_URI=http://remotemachine:11311)
4) Use the source command on naoros.env (e.g. source naoros.env). 
5) Run moveTalk.py (e.g. python moveTalk.py)
You may wish to run this last command with "&" allowing you to log out of the
NAO.

6) Open 3 terminals elsewhere (not on the NAO) and:
7) rosrun wiihead (e.g. rosrun naoExpmnt wiihead)
7) rosrun talk (e.g. rosrun naoExpmnt talk.py)
7) rosrun look (e.g. rosrun naoExpmnt look)

You can now "drive" the NAO. A nunchuck (required) drives the head. The B
trigger moves the robot forward and left and right on the D-pad turn.

########################################
Notes 
########################################
ROS *requires* that every node involved be hosted on a machine that can be
properly getnamed. In our laboratory setup this meant we had to assign the NAO a
public address and modify the NAO's /etc/host and /etc/hostname so that the NAO
"knew" both its name and the name of all ROS nodes involved.
