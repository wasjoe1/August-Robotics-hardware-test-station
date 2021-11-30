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

BASE_NAME=$(basename $0)

if [ -z ${HOME+x} ]; then
    HOME="/home/$USER"
fi

usage() {
    echo -e "Usage: ${BASE_NAME} [-h] [-v] [-r/k] [-t/d]"
    echo -e "-h\t show this Help message"
    echo -e "-v\t Verbose output, equiv. to 'set -x'"
    echo -e "-r\t start rosbag Recording immediately"
    echo -e "-k\t sKip rosbag recording"
    echo -e "-t\t Tar the logs to home folder only"
    echo -e "-d\t tar the logs to home folder then Delete everything"
    # echo -e "WARNING: use these shortcuts with care"
}

while getopts ":rktdhv" opt; do
  case ${opt} in
    r )
        if [ ! -z ${START_RECORDING+x} ]; then
            echo -e "Options -r and -k is mutually exclusive" >&2
            usage && exit 1
        fi
        START_RECORDING=0;;
    k )
        if [ ! -z ${START_RECORDING+x} ]; then
            echo -e "Options -r and -k is mutually exclusive" >&2
            usage && exit 1
        fi
        START_RECORDING=1;;
    t )
        if [ ! -z ${TAR_LOGS+x} ]; then
            echo -e "Options -t and -d is mutually exclusive" >&2
            usage && exit 1
        fi
        TAR_LOGS=0;TRASH_ROS_FOLDER=1;;
    d )
        if [ ! -z ${TAR_LOGS+x} ]; then
            echo -e "Options -t and -d is mutually exclusive" >&2
            usage && exit 1
        fi
        TAR_LOGS=0;TRASH_ROS_FOLDER=0;;
    v    ) set -o xtrace;;
    [h?] ) usage && exit;;
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
    whiptail --yesno "Current device: ${DEVICE_NAME}\nStart rosbag recording now?" 8 78 --defaultno --title "${TUI_TITLE}"
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
        whiptail --yesno "Current device: ${DEVICE_NAME}\nRosbag is now working in the background...\nPress *Stop* when you are done." 8 78 --defaultno --yes-button "Stop" --no-button "Continue" --title "${TUI_TITLE}"
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

# Also dump the django database for reference
# TEMP_DB_DUMP=$(mktemp /tmp/dbdump.XXXXX)
# pushd ~/catkin_ws/src/boothbot/backoffice
# ./manage.py dumpdata --indent 2 > $TEMP_DB_DUMP
# popd

# Prompt if user wants to packup the logs
if [ -z ${TAR_LOGS+x} ]; then
    whiptail --yesno "Rosbag is now *stopped*.\nDo you want me to pack all the logs?\nIf you want to delete ~/.ros, tar(pack) first" 11 78 --defaultno --title "${TUI_TITLE}"
    TAR_LOGS=$?
fi

if [ ${TAR_LOGS} -eq 0 ]; then
    FREE_SPACE=$(df -Pk ~ | tail -1 | awk '{print $4}')
    ROS_DIR_SIZE=$(du -sk $HOME/.ros | cut -f1)
    if [ ${FREE_SPACE:-0} -lt ${ROS_DIR_SIZE:-0} ]; then
        echo "Low on avalible disk space! Avail:${FREE_SPACE}K, ~/.ros:${ROS_DIR_SIZE}K"
        echo "Cleanup ~/.ros and revisit this script"
        exit 1;
    fi
    TAR_NAME="boothbot-log_${DEVICE_NAME}_$(date '+%Y%m%d.%H%M%S').tar"
    tar --ignore-failed-read -cvf ~/${TAR_NAME} /var/log/syslog.* /var/log/boothbot-backoffice/ ~/.ros/ 2>${TEMP_LOG}
    if [ $? -ne 0 ]; then
        echo "Logs & bags packing failed, abort."; exit 1
    fi
    echo "Output tar file is at ~/${TAR_NAME}"
else
    if [ ${TRASH_ROS_FOLDER:-1} -eq 0 ]; then
        echo "If you intend to cleanup, do a tar backup first!"
    fi
    exit 0;
fi

# Prompt if user wants to cleanup
if [ -z ${TRASH_ROS_FOLDER+x} ]; then
    whiptail --yesno "Do you want me to delete EVERYTHING in ~/.ros?\nThis is NOT reversible!" 11 78 --defaultno --title "${TUI_TITLE}"
    TRASH_ROS_FOLDER=$?
fi

if [ ${TRASH_ROS_FOLDER} -eq 0 ]; then
    LATEST_DIR="$(readlink -f $HOME/.ros/log/latest)"
    find .ros/log/ -type d -not -name "$(basename $LATEST_DIR)" -delete
    find .ros -maxdepth 1 -not -name "log" -delete
    [ $? -eq 0 ] && echo -e "Cleanup successful! All except the latest logs are gone."
fi

echo "Script completed successfully."
