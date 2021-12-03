#!/usr/bin/env bash

PC=$1

if [[ "$PC" == "augbooth" ]]; then
    if [[ "$2" == "i5" ]]; then
        sudo rm /etc/NetworkManager/dispatcher.d/remote_access_i5.sh
        sudo cp /home/$PC/catkin_ws/src/boothbot/remote_access/remote_access_i5.sh /etc/NetworkManager/dispatcher.d/
        echo "Large PC copied remote_access_i5.sh to /etc/NetworkManager/dispatcher.d/"
    else
        sudo rm /etc/NetworkManager/dispatcher.d/remote_access_large.sh
        sudo cp /home/$PC/catkin_ws/src/boothbot/remote_access/remote_access_large.sh /etc/NetworkManager/dispatcher.d/
        echo "Large PC copied remote_access_large.sh to /etc/NetworkManager/dispatcher.d/"
    fi
elif [[ "$PC" == "guiding" ]]; then
    sudo rm /etc/NetworkManager/dispatcher.d/remote_access_small.sh
    sudo cp /home/$PC/catkin_ws/src/boothbot/remote_access/remote_access_small.sh /etc/NetworkManager/dispatcher.d/
    echo "Small PC copied remote_access_small.sh to /etc/NetworkManager/dispatcher.d/"
else
    echo "Unknown arg which has to be 'guiding' or 'augbooth'."
fi
