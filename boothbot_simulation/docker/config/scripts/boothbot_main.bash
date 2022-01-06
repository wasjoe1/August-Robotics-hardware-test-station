#!/bin/bash -i
source /home/augbooth/docker/scripts/env.bash

waiting_roscore

stdbuf -o L roslaunch --wait boothbot_main boothbot_main.launch fake_cb:=true using_dtu:=false fake_chassis_usb:=true fixed_lan_ip:=true silent_imu_check:=true
tail -f /dev/null
