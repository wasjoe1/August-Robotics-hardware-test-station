#!/bin/bash -i
source /home/augbooth/docker/scripts/env.bash
cd ${CATKIN_WS_PATH} && catkin build -cs

cd ${CATKIN_WS_PATH}/src/boothbot/backoffice
python -m pip install -e .
./manage.py migrate;
./manage.py collectstatic --noinput

cd ${DEVICE_CONFIG_PATH}/maps
ln -sf 0/empty.yaml default.yaml
