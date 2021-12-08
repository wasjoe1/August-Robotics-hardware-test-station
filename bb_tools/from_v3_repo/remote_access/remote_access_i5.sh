#!/usr/bin/env bash

IF=$1
STATUS=$2

USERNAME="augbooth"
DESIRED_PORT="enp2s0"

if [ "$IF" == "$DESIRED_PORT" ]
then
    case "$2" in
    up)
    logger -s "NM Script up triggered"
    source /home/$USERNAME/catkin_ws/src/boothbot/remote_access/lan_port_monitor.sh $USERNAME $DESIRED_PORT
    ;;
    down)
    logger -s "NM Script down triggered"
    echo "down"
    sudo pkill -9 -f ngrok
    ;;
    pre-up)
    logger -s "NM Script pre-up triggered"
    echo "pre-up"
    ;;
    post-down)
    logger -s "NM Script post-down triggered"
    ;;
    *)
    ;;
    esac
else
    logger -s "Possibly not plugged in at all OR plugged into wrong computer OR wrong port"
fi
