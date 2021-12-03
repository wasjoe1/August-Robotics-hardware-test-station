#!/usr/bin/env bash
# -*- coding: utf-8 -*-

# Make sure roscore is running

CMD="roslaunch guiding_beacon camera_beacon.launch no_tracker:=True"
NEWCMD="rostopic pub -1 /cb_0/cmdword std_msgs/String disable"
RUNCMD="rostopic pub -1 /cb_0/cmdword std_msgs/String enable"
SHOWCMD="rosrun boothbot_perception two_camera_tuning.py"

tmux new-session -d -s bb_session -n two_cameras_alignment
tmux new-window -t bb_session -n two_cameras_alignment_pub
eval `tmux send-keys -t bb_session:two_cameras_alignment "${CMD}" C-m`
gnome-terminal -e "bash -ic '${SHOWCMD}; bash'"
gnome-terminal -e "tmux a -t bb_session:two_cameras_alignment"

echo "------------------------------------------------------------------------------------------"
echo "| Press 'n' to release motor, and pointing camera sets to the wall, then make sure that  |"
echo "| line laser is overlap with the center line of short camera's image, then tune the short|"
echo "| camera so that the center line of long camera's image is overlap with line laser       |"
echo "------------------------------------------------------------------------------------------"
echo "Now will wait for 15 seconds for launching script"
sleep 15
echo "Running..."
echo "取消使能 'n',  使能'r', 退出 'e':"

while true
do
read -rsn1 input
if [ "$input" = "n" ]; then
    eval `tmux send-keys -t bb_session:two_cameras_alignment_pub "${NEWCMD}" C-m`
elif [ "$input" = "r" ]; then
    eval `tmux send-keys -t bb_session:two_cameras_alignment_pub "${RUNCMD}" C-m`
elif [ "$input" = "e" ]; then
    break
else
    echo "Wrong input!"
fi
done

tmux kill-session -t bb_session
