#!/usr/bin/env bash

# Make sure roscore is running

CMD="rosrun boothbot_perception camera_tuning_node.py _short_camera:=False"

# -i is magic :D
gnome-terminal -e "bash -ic '${CMD}; bash'"
