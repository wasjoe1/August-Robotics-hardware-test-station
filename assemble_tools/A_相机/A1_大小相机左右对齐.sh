#!/usr/bin/env bash

# Make sure roscore is running

CMD="roslaunch guiding_beacon two_cameras_alignment.launch with_servo:=False show_frame:=True"

# -i is magic :D
gnome-terminal -e "bash -ic '${CMD}; bash'"
