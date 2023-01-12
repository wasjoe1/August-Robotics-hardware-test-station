#!/bin/bash
source /home/augbooth/swmaster.sh;
stdbuf -o L roslaunch --wait boothbot_portal gs_hub.launch;
