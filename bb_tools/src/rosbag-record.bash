#!/bin/bash

#############################################
#
# Script to help user to record rosbags
#
# author:   Cyrus Chan
# modified: 2021-11-25 1201 HKT
#
#############################################

# set -o xtrace 

usage() {
    echo -e "Usage: $0 [-h] [-r/k] [-t/d]"
    echo -e "-h\t show this Help message"
    echo -e "-r\t start rosbag Recording immediately"
    echo -e "-k\t sKip rosbag recording"
    echo -e "-t\t Tar the logs to home folder only"
    echo -e "-d\t tar the logs to home folder then Delete everything"
    echo -e "WARNING: use these shortcuts with care"
    exit 1;
}

while getopts ":rktdh" opt; do
  case ${opt} in
    r ) 
        [ ! -z ${START_RECORDING+x} ] && echo -e "Options -r and -k is mutually exclusive" && exit 1
        START_RECORDING=0;;
    k ) 
        [ ! -z ${START_RECORDING+x} ] && echo -e "Options -r and -k is mutually exclusive" && exit 1
        START_RECORDING=1;;
    t )
        [ ! -z ${TAR_LOGS+x} ] && echo -e "Options -t and -d is mutually exclusive" && exit 1
        TAR_LOGS=0;TRASH_ROS_FOLDER=1;;
    d ) 
        [ ! -z ${TAR_LOGS+x} ] && echo -e "Options -t and -d is mutually exclusive" && exit 1
        TAR_LOGS=0;TRASH_ROS_FOLDER=0;;
    [h?] ) usage;;
  esac
done
TEMP_LOG=$(mktemp /tmp/rosbag-recorder.XXXXXXXXX.log)
TUI_TITLE="Lionel Rosbag Recorder"

# Setup logs and make sure ROS is up
[ -z ${ROS_DISTRO+x} ] && >&2 echo -e "Is ROS installed on the machine?" && exit 1
source /opt/ros/${ROS_DISTRO}/setup.bash
EXP_501=$(curl ${ROS_MASTER_URI} 2>/dev/null)
if [ $? -ne 0 ]; then
    >&2 echo "Make sure roscore is up, or fix your network settings!"
    exit 1;
fi
if ! hash whiptail; then
    >&2 echo "Make sure whiptail is installed"
    >&2 echo "try: 'sudo apt install whiptail'"
    exit 1;
fi

# Find device name from yaml or use the default "LIONEL"
DEVICE_NAME=$(cat ~/catkin_ws/src/boothbot/common/scripts/common/device_name.yaml 2>/dev/null | grep -oE "\b[A-Za-z0-9]{1,}-[A-Za-z0-9]{1,}-[A-Za-z0-9]{1,}-[0-9]{1,}\b") 
DEVICE_NAME=${DEVICE_NAME:-LIONEL}

# Prompt if user wants to start recording
if [ -z ${START_RECORDING+x} ]; then
    whiptail --yesno "Current device: ${DEVICE_NAME}\nStart rosbag recording now?" 8 78 --defaultno --title ${TUI_TITLE}
    START_RECORDING=$?
fi

if [ ${START_RECORDING} -eq 0 ]; then

    # WORKAROUND: cannot write to the ~/.ros directory
    BAG_FOLDER="${HOME}/.ros/bags/"; mkdir -p ${BAG_FOLDER}
    rosbag record -o "${BAG_FOLDER}/lionel" -a -x "(.*)/keyword(.*)" >${TEMP_LOG} 2>&1 &
    ROSBAG_PID=$!

    # Refresh the tui until user press "stop"
    STOP_RECORDING=1
    while [ ${STOP_RECORDING} -ne 0 ]; do
        whiptail --yesno "Current device: ${DEVICE_NAME}\nRosbag is now working in the background...\nPress *Stop* when you are done." 8 78 --defaultno --yes-button "Stop" --no-button "Continue" --title ${TUI_TITLE}
        STOP_RECORDING=$?
    done

    kill -SIGINT ${ROSBAG_PID} || true
    wait ${ROSBAG_PID}
fi

PGREP_BAGS=$(pgrep -x rosbag)
if [ $? -eq 0 ]; then
    echo "Seems like there is still some active rosbag running..." >&2
    (echo -n "PIDs: " && echo ${PGREP_BAGS} | xargs echo) >&2
    echo "Terminating here, no packing/cleanup performed." >&2
    exit 1
fi

# Prompt if user wants to packup the logs
if [ -z ${TAR_LOGS+x} ]; then
    whiptail --yesno "Rosbag is now *stopped*.\nDo you want me to pack all the logs?" 10 78 --defaultno --title ${TUI_TITLE}
    TAR_LOGS=$?
fi

if [ ${TAR_LOGS} -eq 0 ]; then
    TAR_NAME="boothbot-log_${DEVICE_NAME}_$(date '+%Y%m%d.%H%M%S').tar.gz"
    tar --ignore-failed-read -czf ~/${TAR_NAME} /var/log/syslog.* /var/log/boothbot-backoffice/ ~/.ros/ 2>${TEMP_LOG}
    echo "Output tar file is at ~/${TAR_NAME}"
else
    if [ ${TRASH_ROS_FOLDER:-1} -eq 0 ]; then
        echo "If you intend to cleanup, do a tar backup first!"
    fi
    exit 0;
fi

# Prompt if user wants to cleanup
if [ -z ${TRASH_ROS_FOLDER+x} ]; then
    whiptail --yesno "Do you want me to trash EVERYTHING in ~/.ros?" 10 78 --defaultno --title ${TUI_TITLE}
    TRASH_ROS_FOLDER=$?
fi

if [ ${TRASH_ROS_FOLDER} -eq 0 ]; then
    cd ${HOME} && gio trash ~/.ros/*
    [ $? -eq 0 ] && echo -e "Cleanup successful! If you want to recue the logs,\nfind it from the trash"
fi

echo "Done"