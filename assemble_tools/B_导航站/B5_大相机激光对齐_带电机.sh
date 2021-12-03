#!/usr/bin/env bash
# -*- coding: utf-8 -*-

# Make sure roscore is running

CMD="roslaunch guiding_beacon laser_tuning.launch"
FIXCMD="rostopic pub -1 /gs_0/cmdword std_msgs/String enable"
FREECMD="rostopic pub -1 /gs_0/cmdword std_msgs/String disable"
LASERCMD="rostopic pub -1 /gs_0/cmdword std_msgs/String laseron"
UPDATECMD="rostopic pub -1 /laser_tuning/cmdword std_msgs/String update_dist"

tmux new-session -d -s bb_session -n laser_tuning
tmux new-window -t bb_session -n laser_tuning_pub
eval `tmux send-keys -t bb_session:laser_tuning "${CMD}" C-m`
gnome-terminal -e "tmux a -t bb_session:laser_tuning"

echo "will wait for 15 seconds for launching script"
sleep 15
echo "Running..."
echo "取消使能 'n', 使能 'r', 激光 'l', 更新距离 'u', 退出 'e':"

while true
do
read -rsn1 input
if [ "$input" = "n" ]; then
    eval `tmux send-keys -t bb_session:laser_tuning_pub "${FREECMD}" C-m`
elif [ "$input" = "r" ]; then
    eval `tmux send-keys -t bb_session:laser_tuning_pub "${FIXCMD}" C-m`
elif [ "$input" = "l" ]; then
    eval `tmux send-keys -t bb_session:laser_tuning_pub "${LASERCMD}" C-m`
elif [ "$input" = "u" ]; then
    eval `tmux send-keys -t bb_session:laser_tuning_pub "${UPDATECMD}" C-m`
elif [ "$input" = "e" ]; then
    break
else
    echo "Wrong input!"
fi
done

tmux kill-session -t bb_session
