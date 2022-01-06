#!/bin/bash
source /home/augbooth/docker/scripts/env.bash

waiting_roscore

python /home/augbooth/catkin_ws/src/boothbot-config/common/scripts/common/twisted_server.py;
