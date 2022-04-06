#!/bin/bash -i
source /home/augbooth/docker/scripts/env.bash

waiting_roscore

roslaunch --wait fleet_main start.launch