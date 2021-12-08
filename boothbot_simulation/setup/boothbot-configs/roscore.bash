#!/bin/bash -i
source /opt/ros/melodic/setup.bash;

/home/augbooth/catkin_ws/src/augustbot-tools/bb_tools/docker/scripts/build.bash;
source /home/augbooth/catkin_ws/devel/setup.bash;

stdbuf -o L roscore;