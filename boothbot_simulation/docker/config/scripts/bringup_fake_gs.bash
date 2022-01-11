#!/bin/bash -i
source /home/augbooth/docker/scripts/env.bash

waiting_roscore

stdbuf -o L roslaunch --wait boothbot_driver bringup_fake_gs.launch
tail -f /dev/null
