#!/usr/bin/env bash
source ../scripts/exposure_tuning.sh

echo "Enter distance:"
read dist

EXPOSURE_WINDOW='long_exposure'
CMD="roslaunch guiding_beacon camera_beacon.launch no_tracker:=True"
RUNCMD="rostopic pub -1 /cb_0/cmdword std_msgs/String enable"
NEWCMD="rostopic pub -1 /cb_0/cmdword std_msgs/String disable"
SHOWCMD="rosrun boothbot_perception camera_show_image.py /dev/camera_long"
EXPOCMD="roslaunch guiding_beacon camera_exposure.launch short_camera:=False longc_dist:=${dist} beacon_color:="
CAMERA_PORT="/dev/camera_long"
EXPOSURE_ABSOLUTE=3000
BEACON_COLOR="GREEN"

run_tuning
