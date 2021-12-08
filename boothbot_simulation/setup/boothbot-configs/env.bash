#!/bin/bash -i

# Fake GS required environment variables
export TC_HOST_IP=127.0.0.1
export GS_LAN_PORT=lo
export PTY_DTU=1

# For simulation report path
export CATKIN_WS_PATH="${HOME}/catkin_ws"
export TODAY=`date +%y-%m-%d`
branch_name="$(cd ${CATKIN_WS_PATH}/src/boothbot &&  git rev-parse --abbrev-ref HEAD 2>/dev/null)"
cd -
#branch_name="$(git rev-parse --abbrev-ref HEAD 2>/dev/null)"
export GIT_HEAD=${branch_name}