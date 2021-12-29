#!/bin/bash -i
source /home/augbooth/docker/scripts/env.bash
cd ${CATKIN_WS_PATH} && catkin build -cs

cd ${CATKIN_WS_PATH}/src/boothbot/backoffice;
./manage.py migrate;
./manage.py collectstatic --noinput
