#!/usr/bin/env bash

# Make sure roscore is running
CMD="roslaunch guiding_beacon laser_tuning.launch enable_platform:=True laser_dist:=50"

# -i is magic :D
gnome-terminal -e "bash -ic '${CMD}; bash'"
