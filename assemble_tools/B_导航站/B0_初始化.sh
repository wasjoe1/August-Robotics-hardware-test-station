#!/usr/bin/env bash
# -*- coding: utf-8 -*-

exec < /dev/tty
GSLAUNCH="roslaunch guiding_beacon guiding_station.launch"
FIXCMD="rostopic pub -1 /gs_0/cmdword std_msgs/String enable"
FREECMD="rostopic pub -1 /gs_0/cmdword std_msgs/String disable"
LASERCMD="rostopic pub -1 /gs_0/cmdword std_msgs/String laseron"
ECHOCMD="rostopic echo -c /gs_0/hardware_status/joint"

read -p 'Enter device name(e.g. GS03-GM03LB02-0039): ' device_name

folder_path=~/catkin_ws/src/boothbot/common/scripts/common/device_settings/$device_name
file_path=~/catkin_ws/src/boothbot/common/scripts/common/device_settings/$device_name/device_settings.yaml
name_path=~/catkin_ws/src/boothbot/common/scripts/common/device_name.yaml

if [ -d "$folder_path" ]; then
    if [ -d "/tmp/$device_name" ]; then
        rm /tmp/$device_name
    fi
    echo "Moving old settings to /tmp/ folder"
    mv $folder_path /tmp/
    echo "Moving Done!"
fi
echo "Now copying new settings and setup device_name"
cp -r GS_default_settings $folder_path
echo "name: $device_name" > $name_path

echo "Launching GS Now.."
tmux new-session -d -s bb_session -n initial_GS
tmux new-window -t bb_session -n initial_GS_pub
eval `tmux send-keys -t bb_session:initial_GS "${GSLAUNCH}" C-m`
gnome-terminal -e "bash -ic '${ECHOCMD}; bash'"
gnome-terminal -e "tmux a -t bb_session:initial_GS"

echo "will wait for 15 seconds for launching script"
sleep 15
echo "Running..."
echo "取消使能 'n', 使能 'r', 激光 'l', 退出 'e':"

while true
do
read -rsn1 input
if [ "$input" = "n" ]; then
    eval `tmux send-keys -t bb_session:initial_GS_pub "${FREECMD}" C-m`
elif [ "$input" = "r" ]; then
    eval `tmux send-keys -t bb_session:initial_GS_pub "${FIXCMD}" C-m`
elif [ "$input" = "l" ]; then
    eval `tmux send-keys -t bb_session:initial_GS_pub "${LASERCMD}" C-m`
elif [ "$input" = "e" ]; then
    break
else
    echo "Wrong input!"
fi
done

tmux kill-session -t bb_session

read -p "horizontoal zero_offset:" hor_zero
read -p "vertical zero_offset:" ver_zero
sed -i "15s/0/${hor_zero}/g" $file_path
sed -i "23s/0/${ver_zero}/g" $file_path
