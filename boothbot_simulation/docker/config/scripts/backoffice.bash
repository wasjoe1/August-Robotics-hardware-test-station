#!/bin/bash
source /home/augbooth/docker/scripts/env.bash

waiting_roscore

cd ${CATKIN_WS_PATH}/src/boothbot/backoffice;
./manage.py runserver 0.0.0.0:8000
