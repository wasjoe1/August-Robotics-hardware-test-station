#!/bin/bash -i
source /home/augbooth/docker/scripts/env.bash

cd ${CATKIN_WS_PATH} && catkin build -cs

cd ${CATKIN_WS_PATH}/src/boothbot_fleet/fleet_gui
python3 manage.py migrate --noinput
python3 manage.py collectstatic --noinput --clear --no-post-process