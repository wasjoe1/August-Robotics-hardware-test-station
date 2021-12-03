#!/usr/bin/env bash

# Make sure roscore is running

CMD="rosrun boothbot_driver driving_ratio_check.py"

# -i is magic :D
gnome-terminal -e "bash -ic '${CMD}; bash'"
