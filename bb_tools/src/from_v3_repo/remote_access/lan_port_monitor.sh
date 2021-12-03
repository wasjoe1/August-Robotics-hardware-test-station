#!/bin/bash
# This file will be executed by /etc/NetworkManager/dispatcher.d/remote_access_{large, small}.sh
# once the MiFi is connected. The first argument will be the username.

USERNAME=$1
PORT=$2 # LAN port

if [[ "$USERNAME" != "augbooth" && "$USERNAME" != "guiding" ]]; then
    exit 127
fi

# Check if the network interface is ready
CARRIER_PATH="/sys/class/net/$PORT/carrier"
connected_status=`cat $CARRIER_PATH 2>/dev/null`
if [[ "$?" -ne "0" ]]; then
    exit 127 # The carrier file cannot be read, so exist as failure
else
    ngrok_pid=`pgrep ngrok`
    if [[ "$connected_status" -eq "1" ]]; then
        echo "Port connected"
        if [[ -n "$ngrok_pid" ]]; then
            echo "UP PID IS: $ngrok_pid"
            echo "ngrok already up. SKIP"
        else
            echo "Start running remote_access.py"
            # Run ros setup scripts such that ROS variables are added in env
            source /opt/ros/kinetic/setup.bash
            source /home/$USERNAME/catkin_ws/devel/setup.bash
            # Run the remote access script
            python /home/$USERNAME/catkin_ws/src/boothbot/remote_access/remote_access.py $USERNAME
        fi
    else
        echo "Port disconnected"
        # If there is a PID, and it is disconnected, remove it
        if [[ -n "$ngrok_pid" ]]; then
            sudo pkill -9 -f ngrok
        fi
    fi
fi

sleep 1 # To stagger calls

exit 0 # Exit successfully
