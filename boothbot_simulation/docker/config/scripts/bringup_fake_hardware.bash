#!/bin/bash -i
source /home/augbooth/docker/scripts/env.bash

waiting_roscore

stdbuf -o L roslaunch --wait boothbot_driver boothbot_simulation_hardware.launch
tail -f /dev/null
