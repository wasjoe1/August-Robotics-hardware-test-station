#!/usr/bin/env bash
# When setup rules, please make sure USER and USBDEV are correct.
# USBDEV normally is `/dev/sda1` or `/dev/sdb1`
USER=$1
USBDEV=$2

/home/${USER}/catkin_ws/src/boothbot/log_center/usb_mount.sh ${USER} ${USBDEV} & exit
