#!/bin/bash -i
source /opt/ros/melodic/setup.bash;

/home/augbooth/catkin_ws/src/boothbot/docker/scripts/build.bash;
source /home/augbooth/catkin_ws/devel/setup.bash;

stdbuf -o L roscore;