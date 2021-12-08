#!/usr/bin/env bash

# USB has to be ext4 format (because of symlink)
# Please make sure code is updated in USB. Code has to be in `boothbot` folder

# When runing script, please make sure USER and USBDEV are correct.
# USBDEV normally is `/dev/sda1` or `/dev/sdb1`
USER=$(whoami)
USBDEV=""
BRANCH=""

show_help () {
    echo "[ERROR] Usage: ./sync_code.sh -d /dev/<USBDEV> -b <BRANCH> [-u <USER>]"
}

while getopts "u:d:b:" opt; do
    case "$opt" in
        u)  USER=${OPTARG}
            ;;
        d)  USBDEV=${OPTARG}
            ;;
        b)  BRANCH=${OPTARG}
            ;;
    esac
done

if [[ $USER == "" ]] || [[ $USBDEV == "" ]] || [[ $BRANCH == "" ]] ; then
    show_help
    exit 1
fi

# mount
MOUNTED=/home/${USER}/USBDrive
mkdir ${MOUNTED}
sudo mount ${USBDEV} ${MOUNTED}
sudo chown -R ${USER}:${USER} ${MOUNTED}

echo "USB mounted. Start sync code"
sleep 1

TARGET=/home/${USER}/catkin_ws/src/boothbot
cd ${TARGET}
FOLDER=$(pwd)

if [[ $FOLDER == $TARGET ]]; then
    git remote add gitcode ${MOUNTED}/boothbot/.git
    sleep 0.5
    echo ""
    git pull gitcode ${BRANCH}
    sleep 0.5
    echo ""
    git checkout ${BRANCH}
    echo ""
    git branch --set-upstream-to gitcode/${BRANCH}
    echo ""
    git status
    echo ""
    echo "Code synced. Start unmount USB"
else
    echo "DIDN'T 'cd' to target folder ${TARGET}"
fi

sleep 1
sudo umount ${MOUNTED}
sudo rm -r ${MOUNTED}
echo "DONE!!!"
