#!/usr/bin/env bash
USER=$1
USBDEV=$2

echo "Mount USB..."
# mount
MOUNTED=/home/${USER}/Desktop/USBDrive
mkdir ${MOUNTED}
sudo mount ${USBDEV} ${MOUNTED}
sudo chown -R ${USER}:${USER} ${MOUNTED}
