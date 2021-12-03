#!/usr/bin/env bash

# Please make sure the USB format is ext4

# When runing script, please make sure USER and USBDEV are correct.
# USBDEV normally is `/dev/sda1` or `/dev/sdb1`
USER=$(whoami)
USBDEV=""
DAYS=7

show_help () {
    echo "[ERROR] Usage: ./sync_log.sh -d /dev/<USBDEV> [-s <DAYS>] [-u <USER>]"
}

while getopts "u:d:s:" opt; do
    case "$opt" in
        u)  USER=${OPTARG}
            ;;
        d)  USBDEV=${OPTARG}
            ;;
        s)  DAYS=${OPTARG}
            ;;
    esac
done

if [[ $USER == "" ]] || [[ $USBDEV == "" ]] || [[ $DAYS == "" ]] ; then
    show_help
    exit 1
fi

DAYS=-${DAYS}  # change to negative

echo "Mount USB..."
# mount
MOUNTED=/home/${USER}/USBDrive
mkdir ${MOUNTED}
sudo mount ${USBDEV} ${MOUNTED}
sudo chown -R ${USER}:${USER} ${MOUNTED}

LOCALHOSTNAME=$(hostname)

# Make sure the USB path is currect and `ros_log` folder exists
TARGET=${MOUNTED}/ros_log/${LOCALHOSTNAME}

# Make target folder
mkdir -p ${TARGET}

# Will go through LOGFOLDER and ignore both after find all valid folders
LOGFOLDER=/home/${USER}/.ros/log
LATESTFOLDER=/home/${USER}/.ros/log/latest

echo "Start syncing logs..."
# Find all folder which created today, not hidden files/folders
# `rsync`
for OUTPUT in $(find $LOGFOLDER -maxdepth 1 \( ! -regex './\..*' \)  -mtime $DAYS)
do
    if [[ $OUTPUT != $LOGFOLDER ]] && [[ $OUTPUT != $LATESTFOLDER ]];then
        rsync -a --no-o --no-g  --info=progress2 "${OUTPUT}" "${TARGET}"
        echo $OUTPUT
    fi
done
echo "Sync finished. Umount now..."

sleep 2
sudo umount ${MOUNTED}
sudo rm -r ${MOUNTED}
echo "DONE!!!"
