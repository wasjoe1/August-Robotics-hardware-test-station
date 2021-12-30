#!/bin/bash -i

# Fake GS required environment variables
export TC_HOST_IP=127.0.0.1
export GS_LAN_PORT=lo
export PTY_DTU=1

# For simulation report path
export DOCKER_CONFIG_PATH="/home/augbooth/docker"
export CATKIN_WS_PATH="/home/augbooth/catkin_ws"
export TODAY=`date +%y-%m-%d`

# boothbot_branch="$(cd ${CATKIN_WS_PATH}/src/boothbot &&  git rev-parse --abbrev-ref HEAD 2>/dev/null)"

# ## in case of git is not usable in docker
# if [ -z "${boothbot_branch}" ]
# then
#     boothbot_branch="$(cat ${CATKIN_WS_PATH}/src/boothbot/.git/HEAD|cut -d' ' -f2- -d/|cut -f2- -d/)"
# fi

## if git is not usable us the following command instead
# boothbot_branch="$(cat ${CATKIN_WS_PATH}/src/boothbot/.git/HEAD|cut -d' ' -f2- -d/|cut -f2- -d/)"

# export GIT_HEAD=${boothbot_branch}

# local config path
export DEVICE_CONFIG_PATH="${CATKIN_WS_PATH}/local"


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

source /opt/ros/melodic/setup.bash
source ${CATKIN_WS_PATH}/devel/setup.bash
echo "=============================USAGE======================================="
echo "CONTAINER_NAME:$CONTAINER_NAME"
echo "ROS_MASTER_URI:$ROS_MASTER_URI"
if [ ! -n "$ROS_HOSTNAME" ]; then
    unset ROS_HOSTNAME
else
    echo "  ROS_HOSTNAME:$ROS_HOSTNAME"
fi

if [ ! -n "$ROS_IP" ]; then
    unset ROS_IP
else
    echo "        ROS_IP:$ROS_IP"
fi
