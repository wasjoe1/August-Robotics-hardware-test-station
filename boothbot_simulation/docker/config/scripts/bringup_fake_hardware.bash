#!/bin/bash -i
source /home/augbooth/docker/scripts/env.bash

waiting_roscore

stdbuf -o L roslaunch --wait boothbot_simulation bringup_fake_hardware.launch
tail -f /dev/null
