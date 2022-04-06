#!/bin/bash -i
CATKIN_WS_PATH=${HOME}/catkin_ws

# Collect Device Information and Show
function waiting_roscore () {
    while true;
    do
    OUTPUT=$(curl -s http://localhost:11311)
    if [ -z "$OUTPUT" ]; then
        echo "Waiting for roscore container to finish launching";
    else
        echo "roscore container has started, launching now";
        break
    fi
    sleep 1
    done
}

source /opt/ros/${ROS_DISTRO}/setup.bash
if [[ -f ${CATKIN_WS_PATH}/devel/setup.bash ]]; then
    source ${CATKIN_WS_PATH}/devel/setup.bash
fi