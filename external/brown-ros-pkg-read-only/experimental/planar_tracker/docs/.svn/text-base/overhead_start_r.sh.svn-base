#!/bin/bash
arg=$1
if [ -z "$arg" ] ; then
	echo "usage: $0 [videomode]"
	echo "[videomode] is the videomode (00-04 and 10-16);"
	echo "    see comments in the file for details"
	echo "(unless otherwise told, use 04)"
	echo ""
	exit
fi

# Video modes
#00:640x480@15
#01:640x480@30
#02:640x480@40
#03:640x480@50
#04:640x480@60
#10:320x240@30
#11:320x240@40
#12:320x240@50
#13:320x240@60
#14:320x240@75
#15:320x240@100
#16:320x240@125

source $ROS_ROOT/tools/rosbash/rosbash

echo "*"
echo "*"
echo "**********************************************************"
echo "*          OVERHEAD START SCRIPT - RIGHT CAMERA          *"
echo "*                                                        *"
echo "* Please make sure you set right camera parameters below *"
echo "**********************************************************"
echo "*"
echo "*"

echo 'Starting ps3 camera driver'
sudo modprobe -r gspca-ov534
sudo modprobe gspca-ov534 videomode=$arg

echo "Setting up camera parameters"
echo "*"
echo "*"
echo "Please turn autogain and autoexposure off and set following"
echo "    camera parameters (right camera):"
echo "Exposure: 70"
echo "Main gain: 10"
echo "*"
echo "*"
guvcview -d /dev/video1

#rostopic echo /tag_positions_r &
roslaunch overhead_r.launch
