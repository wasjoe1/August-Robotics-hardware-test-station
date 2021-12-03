#!/usr/bin/env bash

# Make sure roscore is running

CMD="roslaunch guiding_beacon red_line_tracker.py"

# -i is magic :D
gnome-terminal -e "bash -ic '${CMD}; bash'"
