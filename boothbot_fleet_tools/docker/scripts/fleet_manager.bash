#!/bin/bash
source /opt/ros/noetic/setup.bash;
source /home/augbooth/catkin_ws/devel/setup.bash;

# Django Stuff
cd /home/augbooth/catkin_ws/src/boothbot-fleet/fleet_gui;
python3 manage.py runserver 0.0.0.0:8000;
